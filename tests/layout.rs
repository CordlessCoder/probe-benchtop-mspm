//! The DWARF layout walk, against a committed fixture.
//!
//! The fixture is built from a C file whose whole purpose is to be a shape: a nested struct, an
//! array, a bitfield, a union and a pointer, with a plain scalar beside it as the control. Its
//! source is `tests/fixtures/layout.c` and the ELF next to it is the compiled result, committed so
//! the test needs no cross-compiler.
//!
//! **This covers the constant encoding of a member's offset, which is GCC's.** The expression
//! encoding, which is IAR's, is exercised against real images rather than here — no toolchain in
//! this test's reach emits it. `uleb128` has its own unit tests for the half of that path that is
//! arithmetic.

use std::path::Path;

use probe_bench::Symbols;

/// Where the linker put the fixture's compound variable.
const BASE: u64 = 0x9010;

fn fixture() -> Symbols {
    Symbols::load(Path::new("tests/fixtures/layout-gcc.elf")).expect("fixture loads")
}

fn at(symbols: &Symbols, name: &str) -> (u64, u64) {
    let symbol = symbols.get(name).unwrap_or_else(|_| panic!("{name} should resolve"));
    (symbol.address - BASE, symbol.size)
}

/// The offsets are the C compiler's, so this is the test that would catch a walk that reads the
/// right names and puts them in the wrong places — which is what a missing offset decode looks
/// like, and it is not visible from the names alone.
#[test]
fn every_member_lands_where_the_compiler_put_it() {
    let symbols = fixture();

    // A nested struct contributes one path component per level.
    assert_eq!(at(&symbols, "fixtureInstance.inner.flag"), (0, 1));
    assert_eq!(at(&symbols, "fixtureInstance.inner.pair.low"), (2, 2));
    assert_eq!(at(&symbols, "fixtureInstance.inner.pair.high"), (4, 2));

    // An array is expanded by index, and the stride is the element's own size.
    assert_eq!(at(&symbols, "fixtureInstance.counts[0]"), (8, 4));
    assert_eq!(at(&symbols, "fixtureInstance.counts[1]"), (12, 4));
    assert_eq!(at(&symbols, "fixtureInstance.counts[3]"), (20, 4));

    // The field after the bitfields, which is the one that moves if a skipped bitfield were
    // instead given an offset of its own.
    assert_eq!(at(&symbols, "fixtureInstance.after"), (25, 1));

    // A union's members share an offset. Both spellings of the same four bytes are reachable.
    assert_eq!(at(&symbols, "fixtureInstance.overlay.word"), (28, 4));
    assert_eq!(at(&symbols, "fixtureInstance.overlay.halves[0]"), (28, 2));
    assert_eq!(at(&symbols, "fixtureInstance.overlay.halves[1]"), (30, 2));

    // A pointer is a leaf: its own four bytes, and nothing from the far end of it.
    assert_eq!(at(&symbols, "fixtureInstance.pointer"), (32, 4));
}

/// **A bitfield has no address that means the field**, so naming one would hand back a number that
/// reads as its value and is its neighbours'. The walk leaves it out entirely.
#[test]
fn a_bitfield_is_not_addressable_and_so_is_absent() {
    let symbols = fixture();
    assert!(symbols.get("fixtureInstance.narrow").is_err());
    assert!(symbols.get("fixtureInstance.alsoNarrow").is_err());
}

/// A scalar is already reachable under its own name, and a second entry for it would be a second
/// name for one address with nothing to choose between them.
#[test]
fn a_scalar_variable_gains_no_members() {
    let symbols = fixture();
    assert!(symbols.get("plainScalar").is_ok());
    assert_eq!(symbols.containing("plainScalar.").len(), 0);
}

/// The variable itself keeps the size the linker recorded, not one the walk computed. This is the
/// half that must not move: everything else here is derived from it.
#[test]
fn the_base_symbol_is_untouched() {
    let symbols = fixture();
    let base = symbols.get("fixtureInstance").expect("base resolves");
    assert_eq!(base.address, BASE);
    assert_eq!(base.size, 36);
}

/// Nothing synthesised may reach past what the linker says the variable occupies. A member that
/// does means the debug information and the symbol table disagree, and the address computed from
/// the wrong one of those points into whatever variable follows.
#[test]
fn no_member_runs_past_the_end_of_its_variable() {
    let symbols = fixture();
    let base = symbols.get("fixtureInstance").expect("base resolves");
    for (name, member) in symbols.containing("fixtureInstance.") {
        let end = member.address + member.size;
        assert!(
            end <= base.address + base.size,
            "{name} ends at {end:#x}, past {:#x}",
            base.address + base.size
        );
    }
}
