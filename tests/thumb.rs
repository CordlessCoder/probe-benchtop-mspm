//! The Thumb bit is a flag on code and an address bit on data.
//!
//! The fixture is two single-byte variables and a function: the linker has to put one of the bytes
//! at an odd address, and the function's symbol value carries the Thumb bit. Its source is
//! `tests/fixtures/thumb.c` and the ELF next to it is the compiled result, committed so the test
//! needs no cross-compiler.
//!
//! **The two halves have to disagree**, which is why the function is in the fixture at all. A test
//! that only checked the byte would pass against an implementation that never cleared the bit
//! anywhere, and one that only checked the function is what the code had before.

use std::path::Path;

use probe_bench::{Kind, Symbols};

fn fixture() -> Symbols {
    Symbols::load(Path::new("tests/fixtures/thumb-gcc.elf")).expect("fixture loads")
}

/// A one-byte variable at an odd address resolves to that address.
///
/// Clearing its low bit puts it on its neighbour: a plausible value, read from the wrong place,
/// and the same value for both variables. That is what this catches.
#[test]
fn a_byte_at_an_odd_address_keeps_it() {
    let symbols = fixture();
    let first = symbols.get("firstByte").expect("firstByte resolves");
    let second = symbols.get("secondByte").expect("secondByte resolves");

    // The fixture only exercises the case while the linker still packs them. If this fires, the
    // fixture has stopped being one and the rest of the test means nothing.
    assert_eq!(second.address, first.address + 1, "the two bytes are no longer adjacent");
    assert_eq!(second.address % 2, 1, "neither byte landed at an odd address");

    assert_eq!(first.kind, Kind::Data);
    assert_eq!(second.kind, Kind::Data);
    assert_ne!(
        first.address, second.address,
        "two variables resolved to one address, so one of them is read from the other's storage"
    );
}

/// And a function still has it cleared, because there it is a flag and not an address.
///
/// The other half of the pair: a caller reading a `static` should never have to think about the
/// bit, and a caller calling a function sets it back itself.
#[test]
fn a_function_still_has_the_thumb_bit_taken_off() {
    let symbols = fixture();
    let main = symbols.get("main").expect("main resolves");

    assert_eq!(main.kind, Kind::Code);
    assert_eq!(main.address % 2, 0, "a function address kept the Thumb bit");
    assert_eq!(main.address, 0x8000, "the fixture's entry point moved");
}
