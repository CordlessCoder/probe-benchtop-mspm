//! Struct layout from DWARF, so a compound `static` is addressable field by field.
//!
//! [`crate::Symbols`] on its own reaches a variable and nothing inside it. That is enough for a
//! firmware written against this harness, which publishes scalars at their own symbols. It is not
//! enough for a firmware that was not — where the interesting state is already a module-level
//! struct, and asking for it to be flattened into scalars means asking for a rewrite.
//!
//! **Only the type layout is taken from DWARF. The address still comes from the symbol table.**
//! A variable's location is a `DW_AT_location` expression with a dozen forms, most of which do not
//! describe a fixed address at all; its address in the symbol table is one number that the linker
//! has already resolved. So this module answers "what is at byte 12 of this type" and leaves
//! "where does this variable live" to the half that already knew.
//!
//! What comes out is a list of scalar leaves per variable, each a suffix path and a byte offset.
//! [`crate::Symbols::load`] adds them to the same map as everything else, so a caller asking for
//! `timer.config.period` goes through the lookup it would have used for a plain symbol and every
//! consumer — the watch list, the plot, the CLI's listing — works unchanged.
//!
//! **Every offset is normalised to a `.debug_info` offset rather than a unit-relative one**, and
//! that is load-bearing rather than tidy. A type reference arrives in two forms: `DW_FORM_ref4`,
//! which is relative to its own unit, and `DW_FORM_ref_addr`, which is not. IAR emits both — it
//! keeps one copy of a shared type and points at it from every unit that uses it — so a per-unit
//! index resolves the first kind and silently finds nothing for the second. What that looks like
//! from outside is a firmware whose every module singleton has no fields at all.

use std::borrow::Cow;
use std::collections::HashMap;

use gimli::{DebugInfoOffset, EndianSlice, LittleEndian};
use object::{Object, ObjectSection};

/// How deep a *nested* type is followed.
///
/// A struct of structs is ordinary; eight levels of it is a sign the walk has found something it
/// should not be expanding, and a bound is cheaper than finding out which.
///
/// **Only structure counts against this, never an alias.** `uint16_t` is three DIEs deep before
/// the base type appears — `uint16_t`, `__uint16_t`, `short unsigned int` — so charging a typedef
/// to the nesting budget spends it on spelling. A first version did, and the visible effect was
/// that fields two structs down were missing while their shallower siblings were present, which
/// reads as an incomplete walk rather than as an exhausted counter.
const MAX_DEPTH: usize = 8;

/// How many typedefs and qualifiers are followed before giving up.
///
/// Nothing legitimate stacks this many. The bound exists because a malformed or hostile file can
/// name a type that resolves to itself, and the walk has to terminate on one.
const MAX_ALIAS: usize = 32;

/// How many elements of an array are expanded.
///
/// A byte buffer runs to hundreds of elements and nobody watches element 200. The cap keeps one
/// from crowding out the names a person is actually looking for in a completion list.
const MAX_ARRAY: u64 = 32;

/// Total leaves synthesised, across every variable.
///
/// A debug build of a Rust firmware carries type information for everything the executor holds,
/// and expanding all of it costs load time for names nobody asked for. The cap is high enough
/// that no real firmware's own state reaches it.
const MAX_LEAVES: usize = 20_000;

/// One scalar inside a compound variable.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct Leaf {
    /// Appended to the variable's name: `.first.early`, or `[3]`.
    pub suffix: String,
    /// Bytes from the start of the variable.
    pub offset: u64,
    /// Width of the scalar itself.
    pub size: u64,
}

/// What one DIE carries that a layout walk needs.
///
/// The tree is flattened into this in a single pass rather than walked with a cursor, because the
/// walk follows type references, which jump backwards and across units as often as forwards.
struct Node {
    tag: gimli::DwTag,
    name: Option<String>,
    type_ref: Option<DebugInfoOffset>,
    byte_size: Option<u64>,
    location: Option<u64>,
    /// Set on a bitfield, which is why its presence is what excludes one.
    bit_size: bool,
    count: Option<u64>,
    children: Vec<DebugInfoOffset>,
}

impl Node {
    fn new(tag: gimli::DwTag) -> Self {
        Self {
            tag,
            name: None,
            type_ref: None,
            byte_size: None,
            location: None,
            bit_size: false,
            count: None,
            children: Vec::new(),
        }
    }
}

/// Every compound variable's scalar leaves, by variable name.
///
/// A variable whose type is already a scalar produces no entry: the symbol table alone reaches it,
/// and an empty suffix would be a second name for the same address.
pub(crate) fn leaves_by_variable(file: &object::File<'_>) -> HashMap<String, Vec<Leaf>> {
    let mut out = HashMap::new();
    // **Debug information is an aid, not a contract.** A stripped image, an unfamiliar producer or
    // a DWARF version this gimli does not know all mean the same thing here — fewer names — and
    // none of them should stop a session attaching. So every failure below yields what has been
    // collected so far rather than an error.
    let load = |id: gimli::SectionId| -> Result<Cow<'_, [u8]>, gimli::Error> {
        Ok(match file.section_by_name(id.name()) {
            Some(section) => section.uncompressed_data().unwrap_or(Cow::Borrowed(&[])),
            None => Cow::Borrowed(&[]),
        })
    };
    let Ok(sections) = gimli::DwarfSections::load(&load) else {
        return out;
    };
    let dwarf = sections.borrow(|section| EndianSlice::new(section, LittleEndian));

    let (nodes, variables) = index(&dwarf);

    let mut budget = MAX_LEAVES;
    for (name, type_ref) in variables {
        if budget == 0 {
            break;
        }
        let mut leaves = Vec::new();
        walk(&nodes, type_ref, String::new(), 0, 0, &mut leaves, budget);
        if !leaves.is_empty() {
            budget = budget.saturating_sub(leaves.len());
            out.entry(name).or_insert(leaves);
        }
    }
    out
}

/// Flatten every unit's DIEs into one map, and collect the variables declared at the top of each.
///
/// Only variables at the top of a unit are collected. A `static` inside a function is at a deeper
/// level and shares its name with every other function's local of that name, so admitting those
/// would put several unrelated addresses behind one name with nothing to choose between them.
fn index(
    dwarf: &gimli::Dwarf<EndianSlice<'_, LittleEndian>>,
) -> (HashMap<DebugInfoOffset, Node>, Vec<(String, DebugInfoOffset)>) {
    let mut nodes: HashMap<DebugInfoOffset, Node> = HashMap::new();
    let mut variables = Vec::new();

    let mut units = dwarf.units();
    while let Ok(Some(header)) = units.next() {
        let Ok(unit) = dwarf.unit(header) else { continue };
        // Offsets of the ancestors of the entry being read, so a child can be recorded on its
        // parent. Reset per unit, since a unit's tree never runs into the next one's.
        let mut ancestry: Vec<DebugInfoOffset> = Vec::new();
        let mut entries = unit.entries();

        loop {
            // The entry borrows the cursor, so everything wanted from it is taken here and the
            // borrow released before the cursor is asked where it now stands.
            let node = {
                let Ok(Some(entry)) = entries.next_dfs() else { break };
                let mut node = Node::new(entry.tag());
                for attr in entry.attrs() {
                    match attr.name() {
                        gimli::DW_AT_name => {
                            node.name = dwarf
                                .attr_string(&unit, attr.value())
                                .ok()
                                .and_then(|s| s.to_string().ok().map(str::to_owned));
                        }
                        gimli::DW_AT_type => node.type_ref = global_ref(&unit, attr.value()),
                        gimli::DW_AT_byte_size => node.byte_size = attr.udata_value(),
                        gimli::DW_AT_data_member_location => node.location = member_offset(&attr),
                        gimli::DW_AT_bit_size => node.bit_size = true,
                        gimli::DW_AT_count => node.count = attr.udata_value(),
                        // An upper bound is inclusive, so the count is one more. A bound that is
                        // absent means an unsized array, which the caller's cap handles.
                        gimli::DW_AT_upper_bound => node.count = attr.udata_value().map(|n| n + 1),
                        _ => {}
                    }
                }
                node
            };
            let Some(offset) = entries.offset().to_debug_info_offset(&unit.header) else {
                continue;
            };
            let depth = entries.depth();

            // The stack holds this entry's ancestors, so it is trimmed to the new depth before the
            // entry is pushed as the parent of whatever follows it.
            ancestry.truncate(depth.max(0) as usize);

            if node.tag == gimli::DW_TAG_variable
                && depth == 1
                && let (Some(name), Some(type_ref)) = (node.name.clone(), node.type_ref)
            {
                variables.push((name, type_ref));
            }

            if let Some(parent) = ancestry.last()
                && let Some(parent) = nodes.get_mut(parent)
            {
                parent.children.push(offset);
            }
            nodes.insert(offset, node);
            ancestry.push(offset);
        }
    }

    (nodes, variables)
}

/// Where a member sits inside its enclosing type.
///
/// **Two producers, two encodings of the same number.** GCC writes a plain constant; IAR writes a
/// one-instruction location expression, `DW_OP_plus_uconst <offset>`. Reading only the constant
/// form leaves every member of every struct at offset zero — which does not fail, it just puts the
/// whole struct's worth of names on the first field's address.
fn member_offset(attr: &gimli::Attribute<EndianSlice<'_, LittleEndian>>) -> Option<u64> {
    if let Some(constant) = attr.udata_value() {
        return Some(constant);
    }
    let gimli::AttributeValue::Exprloc(expression) = attr.value() else {
        return None;
    };
    plus_uconst(expression.0.slice())
}

/// The offset out of a one-instruction location expression, `DW_OP_plus_uconst <offset>`.
///
/// Only the one opcode is understood, because only the one is ever emitted for a member's
/// position. Anything else describes a position computed at run time, which has no fixed address
/// to hand back.
fn plus_uconst(bytes: &[u8]) -> Option<u64> {
    const DW_OP_PLUS_UCONST: u8 = 0x23;
    match bytes.split_first()? {
        (&DW_OP_PLUS_UCONST, rest) => uleb128(rest),
        _ => None,
    }
}

/// Read one unsigned LEB128, which is how DWARF writes every variable-width integer.
fn uleb128(bytes: &[u8]) -> Option<u64> {
    let mut value = 0u64;
    let mut shift = 0u32;
    for &byte in bytes {
        // A shift past the width of the result would be a malformed or hostile encoding, and
        // wrapping it round would produce a plausible small offset rather than a refusal.
        if shift >= u64::BITS {
            return None;
        }
        value |= u64::from(byte & 0x7f) << shift;
        if byte & 0x80 == 0 {
            return Some(value);
        }
        shift += 7;
    }
    None
}

/// A type reference as a `.debug_info` offset, whichever of the two forms it arrived in.
fn global_ref(
    unit: &gimli::Unit<EndianSlice<'_, LittleEndian>>,
    value: gimli::AttributeValue<EndianSlice<'_, LittleEndian>>,
) -> Option<DebugInfoOffset> {
    match value {
        gimli::AttributeValue::UnitRef(offset) => offset.to_debug_info_offset(&unit.header),
        gimli::AttributeValue::DebugInfoRef(offset) => Some(offset),
        _ => None,
    }
}

/// Follow typedefs and qualifiers to the type that actually has a layout.
fn resolve(nodes: &HashMap<DebugInfoOffset, Node>, offset: DebugInfoOffset) -> Option<&Node> {
    let mut node = nodes.get(&offset)?;
    for _ in 0..MAX_ALIAS {
        if !matches!(
            node.tag,
            gimli::DW_TAG_typedef
                | gimli::DW_TAG_const_type
                | gimli::DW_TAG_volatile_type
                | gimli::DW_TAG_restrict_type
        ) {
            return Some(node);
        }
        node = nodes.get(&node.type_ref?)?;
    }
    None
}

/// Emit every scalar leaf of the type at `offset`, relative to `base`.
fn walk(
    nodes: &HashMap<DebugInfoOffset, Node>,
    offset: DebugInfoOffset,
    prefix: String,
    base: u64,
    depth: usize,
    out: &mut Vec<Leaf>,
    budget: usize,
) {
    if depth > MAX_DEPTH || out.len() >= budget {
        return;
    }
    // A name for a type, or a qualifier on one. Neither changes the layout, so neither adds a path
    // component and neither costs a level of nesting.
    let Some(node) = resolve(nodes, offset) else { return };

    match node.tag {
        gimli::DW_TAG_structure_type | gimli::DW_TAG_union_type | gimli::DW_TAG_class_type => {
            for &child in &node.children {
                let Some(member) = nodes.get(&child) else { continue };
                if member.tag != gimli::DW_TAG_member {
                    continue;
                }
                // **A bitfield has no whole byte of its own**, so there is no address this crate
                // could hand back that means the field rather than its neighbours. Naming it and
                // returning the storage unit would read as the field's value and usually not be
                // one, so it is left out entirely.
                if member.bit_size {
                    continue;
                }
                let Some(inner) = member.type_ref else { continue };
                // A union's members share offset zero, and an anonymous member contributes no
                // component — which is what makes an anonymous union's fields appear at the
                // enclosing struct's level, where the C that declared them expects them.
                let at = base + member.location.unwrap_or(0);
                let path = match &member.name {
                    Some(name) => format!("{prefix}.{name}"),
                    None => prefix.clone(),
                };
                walk(nodes, inner, path, at, depth + 1, out, budget);
            }
        }

        gimli::DW_TAG_array_type => {
            let Some(inner) = node.type_ref else { return };
            let stride = sizeof(nodes, inner, 0).unwrap_or(0);
            if stride == 0 {
                return;
            }
            // The count sits on a subrange child rather than on the array itself.
            let count = node
                .children
                .iter()
                .filter_map(|child| nodes.get(child))
                .find(|child| child.tag == gimli::DW_TAG_subrange_type)
                .and_then(|child| child.count)
                .or_else(|| node.byte_size.map(|bytes| bytes / stride))
                .unwrap_or(0);
            for index in 0..count.min(MAX_ARRAY) {
                walk(
                    nodes,
                    inner,
                    format!("{prefix}[{index}]"),
                    base + index * stride,
                    depth + 1,
                    out,
                    budget,
                );
            }
        }

        // The leaves. A pointer is one deliberately: its target is not part of this variable, and
        // following it would not terminate on a type that points at itself.
        gimli::DW_TAG_base_type | gimli::DW_TAG_pointer_type | gimli::DW_TAG_enumeration_type => {
            // An empty prefix means the variable was a scalar all along, and the symbol table
            // already reaches it under its own name.
            if prefix.is_empty() {
                return;
            }
            if let Some(size) = node.byte_size {
                out.push(Leaf {
                    suffix: prefix,
                    offset: base,
                    size,
                });
            }
        }

        _ => {}
    }
}

/// The size of a type, following the names and qualifiers that do not have one of their own.
fn sizeof(nodes: &HashMap<DebugInfoOffset, Node>, offset: DebugInfoOffset, depth: usize) -> Option<u64> {
    if depth > MAX_DEPTH {
        return None;
    }
    let node = resolve(nodes, offset)?;
    if let Some(size) = node.byte_size {
        return Some(size);
    }
    // An array records no size of its own in some producers, so it is the one case that has to be
    // computed rather than read.
    if node.tag == gimli::DW_TAG_array_type {
        let inner = node.type_ref?;
        let stride = sizeof(nodes, inner, depth + 1)?;
        let count = node
            .children
            .iter()
            .filter_map(|child| nodes.get(child))
            .find(|child| child.tag == gimli::DW_TAG_subrange_type)
            .and_then(|child| child.count)?;
        return Some(stride * count);
    }
    sizeof(nodes, node.type_ref?, depth + 1)
}

/// **The fixture in `tests/layout.rs` is GCC's, and GCC writes a member's offset as a plain
/// constant.** So it exercises none of the expression decoding below, and removing that decoding
/// entirely leaves all five of those tests passing — checked, rather than assumed. These are what
/// stands behind the encoding IAR uses, and they are unit tests for that reason rather than for
/// tidiness.
#[cfg(test)]
mod tests {
    use super::*;

    /// The whole expression form, opcode included.
    #[test]
    fn plus_uconst_reads_the_expression_iar_writes() {
        // `DW_OP_plus_uconst 0`, which is a first member and the case most easily confused with
        // an absent attribute.
        assert_eq!(plus_uconst(&[0x23, 0x00]), Some(0));
        assert_eq!(plus_uconst(&[0x23, 0x04]), Some(4));
        // Past a single byte of operand, where the continuation bit starts mattering.
        assert_eq!(plus_uconst(&[0x23, 0x80, 0x01]), Some(128));
    }

    /// Any other opcode is a position this crate cannot reduce to an address, and guessing one
    /// would put a member somewhere plausible and wrong.
    #[test]
    fn plus_uconst_refuses_any_other_expression() {
        assert_eq!(plus_uconst(&[]), None);
        // `DW_OP_constu 4`, a different way to say a number, deliberately not accepted.
        assert_eq!(plus_uconst(&[0x10, 0x04]), None);
        // The opcode with no operand at all.
        assert_eq!(plus_uconst(&[0x23]), None);
    }

    /// The encoding DWARF uses for every variable-width integer, so a wrong answer here is a wrong
    /// member offset — a plausible number pointing at the wrong bytes.
    #[test]
    fn uleb128_decodes_the_encoding_dwarf_writes() {
        assert_eq!(uleb128(&[0x00]), Some(0));
        assert_eq!(uleb128(&[0x04]), Some(4));
        assert_eq!(uleb128(&[0x7f]), Some(127));
        // The first value needing a continuation byte, which is where a single-byte reader breaks.
        assert_eq!(uleb128(&[0x80, 0x01]), Some(128));
        assert_eq!(uleb128(&[0xe5, 0x8e, 0x26]), Some(624_485));
    }

    /// Trailing bytes are the normal case: the offset is one operand inside a longer expression.
    #[test]
    fn uleb128_stops_at_the_terminator_and_ignores_the_rest() {
        assert_eq!(uleb128(&[0x04, 0xff, 0xff]), Some(4));
    }

    /// An encoding that never terminates, and one that would overflow the result. Both have to
    /// refuse rather than wrap, because a wrapped offset is a small plausible one.
    #[test]
    fn uleb128_refuses_what_it_cannot_represent() {
        assert_eq!(uleb128(&[]), None);
        assert_eq!(uleb128(&[0x80, 0x80, 0x80]), None);
        assert_eq!(uleb128(&[0xff; 16]), None);
    }
}
