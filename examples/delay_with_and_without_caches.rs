//! What the flash prefetcher and caches are worth, in processor cycles.
//!
//! A delay loop written as three cycles an iteration costs five with them off, and nothing reports
//! it — the register that says so is not restored by the flash algorithm and not restored by a
//! CPU-only reset, so an image flashed by a debugger runs slow and no error is raised. This puts a
//! number on that.
//!
//! Takes an image that **times a loop against SysTick and never configures `CPUSS.CTL`**, plus the
//! names of the two symbols it publishes: the register as that boot found it, and the cycle count.
//! An image that configures the register would report the same number twice.
//!
//! Two boots, and the difference between them is the whole measurement:
//!
//! 1. Straight after programming, where the algorithm's `Init` has left the caches off and nothing
//!    has put them back.
//! 2. After `reset_and_halt`, which arms `DEMCR.VC_CORERESET` and so takes the MSPM0 sequence's
//!    system-reset path — the one that does restore the register.
//!
//! A plain `Core::reset` is deliberately not used: it is a CPU-only reset, it does not restore the
//! register, and both boots would then read the same.
use std::time::Duration;

use probe_bench::{Attach, Bench, Verify};

/// Long enough for the image to reach the loop and publish its count.
const SETTLE: Duration = Duration::from_millis(300);

fn main() -> anyhow::Result<()> {
    let mut args = std::env::args().skip(1);
    let elf = std::path::PathBuf::from(args.next().expect("the timing ELF"));
    let register = args.next().expect("the symbol holding CPUSS.CTL at entry");
    let cycles = args.next().expect("the symbol holding the cycle count");

    let attach = Attach {
        chip: "MSPM0L1306".to_owned(),
        probe: std::env::var("PROBE_RS_PROBE").ok(),
        verify: Verify::Skip,
        ..Default::default()
    };
    let mut bench = Bench::attach(&attach, &elf)?;

    bench.program(&elf)?;
    std::thread::sleep(SETTLE);
    let (off_ctl, off) = read(&mut bench, &register, &cycles)?;
    println!("as the flash left it   CPUSS.CTL {off_ctl:#010x}   {off} cycles");

    bench.reset_and_halt(Duration::from_secs(1))?;
    bench.resume()?;
    std::thread::sleep(SETTLE);
    let (on_ctl, on) = read(&mut bench, &register, &cycles)?;
    println!("after a system reset   CPUSS.CTL {on_ctl:#010x}   {on} cycles");

    if off_ctl == on_ctl {
        println!("\nboth boots found the same register value, so this measured nothing.");
        return Ok(());
    }
    println!("\ncaches off costs {:.3}x", f64::from(off) / f64::from(on.max(1)));
    Ok(())
}

fn read(bench: &mut Bench, register: &str, cycles: &str) -> Result<(u32, u32), probe_bench::Error> {
    Ok((bench.peek::<u32>(register)?, bench.peek::<u32>(cycles)?))
}
