# probe-bench

A small library for driving a microcontroller over a debug probe while its firmware runs.

It sits on [probe-rs](https://probe.rs) and adds the things a bench tool wants and a debugger does
not: read and write a symbol by name, drive a pin, flash an image and know whether it needed
flashing, decode a defmt log, and hold the core across a sequence instead of acquiring it per
access.

## What it is for

Changing a constant in a running firmware and reading a value back, without reflashing. That loop
is most of what an embedded bench session is, and doing it over SWD costs nothing on the target: no
protocol, no framing, no command handler. The ELF's symbol table is the schema.

```rust
use probe_bench::{Attach, Bench, Flashed, Target};

// `chip` is the one field with no useful default.
let attach = Attach { chip: "MSPM0L1306".to_owned(), ..Attach::default() };
let mut bench = Bench::attach(&attach, "firmware.elf".as_ref())?;

// A symbol is an address, and an address is a word.
let before: u32 = bench.peek("blink_period_ms")?;
bench.poke("blink_period_ms", 12u32)?;

// Take the core once for a sequence, rather than a debug-port handshake per access.
let mut held = bench.hold()?;
let a: u32 = held.peek("reading_a")?;
let b: u32 = held.peek("reading_b")?;
```

## What it knows and does not know

**It knows nothing about any particular firmware.** It is handed an ELF path and a symbol name.
Which symbols exist, what they mean and what a sweep of one is for all belong in the caller. That
boundary is why it lives outside any firmware repository, and it is what lets more than one project
use it.

It does know about **MSPM0 silicon**. GPIO through `PINCM` and the application mailbox on SEC-AP
live in their own modules; `CPUSS.CTL`, the SYSCTL power registers and the NONMAIN/BCR layout appear
in examples. Nothing else depends on any of it.

**How far that reaches, checked rather than assumed.** The GPIO block is at `0x400A_0000` plus
`0x2000` per port on all 43 parts in the catalog, and DEBUGSS is access port 2 with the same
register offsets and the same base on all of them — that one is stated in four separate reference
manuals, not inferred from a board.

**The pin-to-`PINCM` mapping is the part that is not family-wide, and it is a table for a reason.**
It equals the pin number plus one on 14 parts; on the other 29 the difference ranges from −27 to
+78, and a second port continues the same numbering rather than restarting. So there is no
arithmetic to be clever with, and a part the table does not know is an error rather than a guess —
guessing would silently mux the wrong pad on two thirds of the family. `src/mspm0_parts.rs` is
generated from the mspm0-data catalog by `tools/generate_iomux.py`.

One value here really is one board's: the SEC-AP `IDR`. No reference manual states it, so it is a
measurement rather than a constant, and it is used as an identity read rather than something to
compare against.

## Flashing

A flash compares before it writes, and says which happened:

```rust
match bench.program(elf)? {
    Flashed::Written => println!("wrote it"),
    Flashed::AlreadyThere => println!("it was already there"),
}
```

The comparison costs about a fifth of a write, and the part is reset either way — so nothing but
the time differs. [`FlashOptions`] has the rest: skipping the erase on a part known to be erased,
skipping the read-back, and erasing the whole chip rather than the sectors the image occupies.

Two of those are claims about the part rather than preferences, and one combination is refused:
flash cells only clear bits, so skipping both the erase and the read-back leaves nothing to catch a
wrong assumption.

## Status

Used daily against Cortex-M0+ parts and shaped by that. The API is not stable, there is no release
on crates.io, and `publish = false` says so.

## Licence

MIT or Apache-2.0, at your option.
