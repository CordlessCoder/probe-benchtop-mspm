//! Drive a pin on a part with no firmware at all.
//!
//! **The demo, and the thing that had to be fixed for it to work.** `mspm0_gpio` writes the pin's
//! mux and the bank's output registers, and until [`mspm0_gpio::power_on`] existed it assumed the
//! bank was powered — which an application's `init` does and a blank part has never done. Every
//! write went to an isolated peripheral, was dropped, and returned `Ok`.
//!
//! # Why this works at all on a blank part
//!
//! A blank MSPM0 faults its core as soon as it is released, and the access port goes down with it.
//! **The core is never released here.** `reset_and_halt` stops it at the reset vector before the
//! boot ROM's jump into erased flash, so nothing faults and the debug port stays up — and the GPIO
//! peripheral is a bus slave the debugger reaches without the CPU's help.
//!
//! # It erases the part, and only when told
//!
//! Pass `--erase` and it clears MAIN. Without it, it attaches to whatever is there and drives the
//! pin anyway, which is the same demonstration on a part somebody has not blanked.
//!
//! Recovery is a reflash in a later session, and it needs `--allow-erase-all` because a blank part's
//! access port is only reachable through the boot ROM's mass erase.
use std::time::Duration;

use probe_bench::{Attach, Verify, mspm0_gpio};

/// `GPIOA.DOE31_0`, which says whether the drive took.
const DOE: u64 = 0x400A_12C0;

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let erase = args.iter().any(|a| a == "--erase");
    let mut plain = args.iter().filter(|a| !a.starts_with("--"));
    // **Only for its symbol table, which nothing here reads.** `Bench::attach` wants an ELF and this
    // has no use for one — `Verify::Skip` means it is never compared against the part, and a blank
    // part would fail any comparison anyway. Any ELF for this chip will do.
    let elf = std::path::PathBuf::from(plain.next().expect("an ELF, for the API's sake"));
    let pin: u8 = plain.next().expect("a pin number").parse()?;
    let pin = mspm0_gpio::Pin(pin);

    let attach = Attach {
        chip: "MSPM0L1306".to_owned(),
        probe: std::env::var("PROBE_RS_PROBE").ok(),
        // There is no image to check against, and after an erase there is nothing at all.
        verify: Verify::Skip,
        // A blank part's access port answers only through the boot ROM's mass erase.
        allow_erase_all: true,
        ..Default::default()
    };
    let mut bench = probe_bench::Bench::attach(&attach, &elf)?;

    if erase {
        println!("erasing MAIN…");
        bench.erase()?;
        println!("erased");
    }

    // Stop before the boot ROM jumps into erased flash. A core that runs there faults, and the
    // access port goes with it.
    bench.reset_and_halt(Duration::from_secs(1))?;
    println!("halted at the reset vector");

    let powered = mspm0_gpio::power_on(&mut bench)?;
    println!("GPIOA {}", if powered { "was not powered, and is now" } else { "was already powered" });

    mspm0_gpio::drive(&mut bench, pin, true)?;
    let doe = bench.read_u32(DOE)?;
    println!(
        "PA{} driven high: DOE {doe:#010x}, the bit {}",
        pin.0,
        if doe & (1 << pin.0) != 0 { "took" } else { "was dropped" }
    );

    bench.leave_halted();
    println!("\nleft halted and driving. Reflash to get an image back on the part.");
    Ok(())
}
