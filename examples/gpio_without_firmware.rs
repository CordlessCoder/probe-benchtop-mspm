//! Can a pin be driven over SWD with no firmware running?
//!
//! `mspm0_gpio` writes the pin's mux and the bank's output registers and nothing else, on the
//! assumption that the bank is already powered — which it is whenever an application has run its
//! HAL's `init`. **On a part that has not run one, it is not**, and every write goes to an isolated
//! peripheral and is dropped. The module's own documentation says it "needs nothing from the
//! firmware", and that is true of a running image and not of a bare part.
//!
//! This measures the difference without erasing anything. A `reset_and_halt` leaves the core stopped
//! at the reset vector with no application code executed, which is the same condition as a blank
//! part for everything below the CPU — so the bank is in its reset state either way.
//!
//! Three phases, and the middle one is the finding:
//!
//! 1. As found, with the image running: `PWREN`, then drive a pin and read `DOE` back.
//! 2. Halted at reset, nothing run: the same, and this is where the write is expected to vanish.
//! 3. Still halted, with the bank powered by hand first.
//!
//! **`DOE` read back is the measurement**, not the pin. A dropped write leaves the bit clear, and
//! that needs no wiring and no scope.
//!
//! Pass a pin that is safe to drive on whatever board this runs on.
use std::time::Duration;

use probe_bench::{Attach, Bench, Held, Target, Verify, mspm0_gpio};

/// `GPIOA`'s GPRCM, from the metapac: the block is at `+0x800` and `PWREN` at `+0x00`.
///
/// The bank's reset is at `+0x04` and is deliberately not written: `PWREN` alone is enough, which is
/// what the third phase below establishes, and asserting reset would clear the pin state of an
/// application that owns pins in this bank.
const PWREN: u64 = 0x400A_0800;
/// `PWREN.KEY` is `0x26` in bits 31:24 and `ENABLE` is bit 0.
const PWREN_ON: u32 = 0x2600_0001;
/// `GPIOA.DOE31_0`, which says whether the drive took.
const DOE: u64 = 0x400A_12C0;

fn main() -> anyhow::Result<()> {
    let mut args = std::env::args().skip(1);
    let elf = std::path::PathBuf::from(args.next().expect("an ELF"));
    let pin: u8 = args.next().expect("a pin number").parse()?;
    let pin = mspm0_gpio::Pin(pin);

    let attach = Attach {
        chip: "MSPM0L1306".to_owned(),
        probe: std::env::var("PROBE_RS_PROBE").ok(),
        verify: Verify::Sampled,
        ..Default::default()
    };
    let mut bench = Bench::attach(&attach, &elf)?;

    // **Reset and run it, rather than assuming.** Attaching does not resume, and a previous session
    // may have left the core halted at the reset vector — in which case this row would be the same
    // condition as the next one and would read as a control while being no such thing.
    bench.reset()?;
    std::thread::sleep(Duration::from_millis(300));
    let status = bench.status()?;
    println!("with the image running ({status:?})");
    anyhow::ensure!(
        !status.is_halted(),
        "the image is not running, so there is no control here"
    );
    report(&mut bench.hold()?, pin)?;

    bench.reset_and_halt(Duration::from_secs(1))?;
    println!("\nhalted at the reset vector, nothing run");
    report(&mut bench.hold()?, pin)?;

    // **`PWREN` alone, no reset.** Asserting a bank's reset is destructive on a part where an
    // application owns pins in it, so if powering is enough on its own then the fix costs one write
    // and can be safe to do unconditionally.
    println!("\nsame, with PWREN written and no reset");
    bench.write_u32(PWREN, PWREN_ON)?;
    // The registers behind `PWREN` stay isolated for a few ULPCLK cycles; a write inside that
    // window is dropped, which is the failure this whole example is about.
    std::thread::sleep(Duration::from_millis(1));
    report(&mut bench.hold()?, pin)?;

    let _ = mspm0_gpio::release(&mut bench.hold()?, pin);
    bench.leave_halted();
    println!("\nleft halted. Reset or reflash to get the image running again.");
    Ok(())
}

fn report(bench: &mut Held<'_>, pin: mspm0_gpio::Pin) -> anyhow::Result<()> {
    println!("  PWREN            {:#010x}", bench.read_u32(PWREN)?);
    let _ = mspm0_gpio::drive(bench, pin, true);
    let doe = bench.read_u32(DOE)?;
    let took = doe & (1 << pin.0) != 0;
    println!(
        "  DOE after drive  {doe:#010x}   the bit {}",
        if took { "took" } else { "was dropped" }
    );
    Ok(())
}
