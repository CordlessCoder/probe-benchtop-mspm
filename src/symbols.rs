//! The ELF's symbol table, which is the whole schema.
//!
//! There is no protocol between host and target here, and no generated header kept in step by hand.
//! A parameter is a `static` in the firmware; the harness looks its name up and writes it. The
//! symbol table cannot drift from the image because it *is* the image.
//!
//! **Where the image carries debug information, the schema reaches inside a compound variable
//! too.** A firmware that keeps its state in a module-level struct rather than in one scalar per
//! reading is then addressable field by field, under a dotted name, without being rewritten to
//! suit this harness. [`crate::layout`] has how, and the addresses still come from the table here.

use std::collections::HashMap;
use std::path::Path;

use object::{Object, ObjectSymbol};

use crate::Error;

/// One symbol: where it lives, how big the linker says it is, and which kind it is.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Symbol {
    /// Target address, with the Thumb bit already cleared.
    pub address: u64,
    /// Size in bytes, from the ELF. Zero when the producer did not record one.
    pub size: u64,
    /// Whether this names data, code, or something the ELF did not classify.
    pub kind: Kind,
}

/// What a symbol names.
///
/// **This is what lets a host offer a firmware's watchable state without being told a prefix.**
/// The alternative is a guess from the address — anything at or above the SRAM base is data — and
/// that is wrong for a target whose RAM is somewhere else, and wrong again for a constant in
/// flash that is worth reading.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Kind {
    /// A variable. What a watch list is made of.
    Data,
    /// A function.
    Code,
    /// Neither, or not recorded. A section or file symbol, and some producers' notion of a label.
    Other,
}

/// Whether a name belongs to the toolchain rather than to whoever wrote the firmware.
///
/// A leading `?` is reserved by IAR for compiler and assembler symbols, which is why its literal
/// pools and anonymous constants all carry one. Nothing a person declared starts with it.
fn is_reserved(name: &str) -> bool {
    name.starts_with('?')
}

/// Every named symbol in an ELF, by name.
pub struct Symbols {
    by_name: HashMap<String, Symbol>,
}

impl Symbols {
    /// Read an ELF and index every symbol that has a name.
    pub fn load(elf: &Path) -> Result<Self, Error> {
        let bytes = std::fs::read(elf).map_err(|source| Error::ElfRead {
            path: elf.to_path_buf(),
            source,
        })?;
        let file = object::File::parse(&*bytes).map_err(|source| Error::ElfParse {
            path: elf.to_path_buf(),
            source,
        })?;

        let mut by_name = HashMap::new();
        for symbol in file.symbols() {
            let Ok(name) = symbol.name() else { continue };
            if name.is_empty() {
                continue;
            }
            // ARM mapping symbols share addresses with real ones and would shadow them.
            if name.starts_with('$') {
                continue;
            }
            // The Thumb bit is set on function symbols and is not part of the address. It is
            // cleared here rather than at every use, because a caller reading a `static` should
            // never have to think about it and a caller calling a function needs it set back —
            // which is that caller's business, and is documented where it arises.
            by_name.insert(
                name.to_owned(),
                Symbol {
                    address: symbol.address() & !1,
                    size: symbol.size(),
                    kind: match symbol.kind() {
                        object::SymbolKind::Data => Kind::Data,
                        object::SymbolKind::Text => Kind::Code,
                        _ => Kind::Other,
                    },
                },
            );
        }

        // **The members are added under the table, never over it.** A synthesised `a.b` can only
        // collide with a real symbol that already has a dot in its name, and where one does the
        // linker's entry is the one that means something — so the real symbol wins and the walk
        // loses a name nobody could have used anyway.
        for (variable, leaves) in crate::layout::leaves_by_variable(&file) {
            let Some(base) = by_name.get(&variable).copied() else {
                continue;
            };
            for leaf in leaves {
                // A member reaching past the end of what the linker recorded means the debug
                // information and the symbol table disagree about this variable, and an address
                // computed from the half that is wrong is a plausible number pointing at another
                // variable. Dropping it is the only safe answer available here.
                if base.size != 0 && leaf.offset + leaf.size > base.size {
                    continue;
                }
                by_name.entry(format!("{variable}{}", leaf.suffix)).or_insert(Symbol {
                    address: base.address + leaf.offset,
                    size: leaf.size,
                    // Data by construction: the walk emits scalar leaves and nothing else.
                    kind: Kind::Data,
                });
            }
        }

        Ok(Self { by_name })
    }

    /// No symbols at all, for a session opened without an ELF.
    ///
    /// **A blank part has no image to read symbols from**, and driving a pin or reading a status
    /// register does not need any. This is what [`crate::Bench::attach_bare`] carries so those
    /// operations can reach a part that a symbol-bearing attach could not.
    #[must_use]
    pub fn none() -> Self {
        Self {
            by_name: HashMap::new(),
        }
    }

    /// Look one up.
    pub fn get(&self, name: &str) -> Result<Symbol, Error> {
        // **An empty table is a different mistake from a missing symbol**, and saying "no symbol
        // named x" for it sends the reader looking for a typo in a name that was never going to be
        // found. An ELF with no symbols at all is not a case worth separating: it fails the same
        // way and for the same reason.
        if self.by_name.is_empty() {
            return Err(Error::NoSymbolTable { name: name.to_owned() });
        }
        self.by_name.get(name).copied().ok_or_else(|| Error::NoSuchSymbol {
            name: name.to_owned(),
            near: self.nearest(name),
        })
    }

    /// Every symbol whose name contains `needle`, sorted. For a CLI that lists what is settable.
    pub fn containing(&self, needle: &str) -> Vec<(&str, Symbol)> {
        let mut found: Vec<_> = self
            .by_name
            .iter()
            .filter(|(name, _)| name.contains(needle))
            .map(|(name, symbol)| (name.as_str(), *symbol))
            .collect();
        found.sort_by_key(|(name, _)| *name);
        found
    }

    /// Every data symbol of known size whose name contains `needle`, sorted.
    ///
    /// **An empty `needle` is the whole watchable surface**, which is what a firmware that was not
    /// written for this harness offers: it has no prefix to filter on, and its state is whatever
    /// its modules happen to keep. Code is excluded because a function's address is not something
    /// a watch list can read a value from, and it would otherwise outnumber the variables.
    ///
    /// **A size of zero is excluded for the same reason, and it is not a rare case.** A compiler
    /// emits data symbols for its own literal pools, and at least one emits hundreds of them — all
    /// classified as data, all of size zero, all named alike. Nothing can read a value of unknown
    /// width, so they are not watchable however they are classified, and admitting them buries the
    /// variables a person came to find.
    ///
    /// **Compiler-reserved names go too**, for the same reason and by the same rule that already
    /// drops ARM mapping symbols from the table: a leading `?` is reserved by one toolchain for its
    /// own symbols, and its anonymous constants arrive under it in quantity.
    ///
    /// The consequence to know about: a producer that records no sizes at all would offer nothing
    /// here, and neither would one that begins its user symbols with a reserved character.
    /// [`Self::containing`] is the unfiltered view for those cases, and every one of these symbols
    /// is still resolvable by name through [`Self::get`] — this filters a listing, not the table.
    pub fn data_containing(&self, needle: &str) -> Vec<(&str, Symbol)> {
        let mut found: Vec<_> = self
            .by_name
            .iter()
            .filter(|(name, symbol)| {
                symbol.kind == Kind::Data && symbol.size != 0 && !is_reserved(name) && name.contains(needle)
            })
            .map(|(name, symbol)| (name.as_str(), *symbol))
            .collect();
        found.sort_by_key(|(name, _)| *name);
        found
    }

    pub fn len(&self) -> usize {
        self.by_name.len()
    }

    pub fn is_empty(&self) -> bool {
        self.by_name.is_empty()
    }

    /// A few names sharing the longest prefix with `name`, for the error message.
    ///
    /// A typo in a symbol name is the most likely way to reach `NoSuchSymbol`, and a bare "not
    /// found" against a table of several thousand is the least useful thing to say about one.
    fn nearest(&self, name: &str) -> Vec<String> {
        let prefix_len = |candidate: &str| name.bytes().zip(candidate.bytes()).take_while(|(a, b)| a == b).count();
        let best = self.by_name.keys().map(|k| prefix_len(k)).max().unwrap_or(0);
        // Nothing in common is not a suggestion, it is noise.
        if best < 3 {
            return Vec::new();
        }
        let mut near: Vec<String> = self
            .by_name
            .keys()
            .filter(|k| prefix_len(k) == best)
            .take(8)
            .cloned()
            .collect();
        near.sort();
        near
    }
}
