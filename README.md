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
use probe_bench::{Attach, Bench};

let mut bench = Bench::attach(&Attach::default(), "firmware.elf".as_ref())?;

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

It does know a fair amount about **MSPM0 silicon**, because that is what it has been used against:
GPIO through `PINCM`, the application mailbox on SEC-AP, `CPUSS.CTL`, the SYSCTL power registers,
and the NONMAIN/BCR layout. Those live in their own modules and nothing else depends on them.

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
