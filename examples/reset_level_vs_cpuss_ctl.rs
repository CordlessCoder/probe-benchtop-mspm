//! Does a CPU-only reset restore `CPUSS.CTL` on an MSPM0, or only a system reset?
//!
//! The MSPM0 debug sequence takes a SYSCTL `SYSRST` path instead of `AIRCR.SYSRESETREQ` **only when
//! `DEMCR.VC_CORERESET` is set** — so `Core::reset_and_halt` gets the system reset and a bare
//! `Core::reset` gets the CPU-only one. That gate is the A/B, and it needs no rebuild.
//!
//! # Why an inert image is required
//!
//! A firmware that configures the prefetcher at start-up overwrites the answer microseconds after
//! the reset that produced it, so *every* row would read as restored. The image this flashes must
//! never write `CPUSS.CTL`. Pass one that does nothing.
//!
//! # What it does
//!
//! 1. Programs the inert image, so nothing running can touch the register.
//! 2. Writes one flash word with no reset after it, which runs the flash algorithm and leaves
//!    `CPUSS.CTL` at whatever its `Init` set.
//! 3. **Waits, and reads again.** A part resetting on its own — a watchdog, a fault loop — would
//!    restore the register and every row below would read as restored for the wrong reason. This is
//!    the control that rules it out, and it is the only part of this that is not obvious.
//! 4. Clears `DEMCR.VC_CORERESET` by hand, resets, halts, reads. That is the CPU-only reset.
//! 5. Puts the register back to the algorithm's state and does it again through `reset_and_halt`,
//!    which arms the catch. That is the system reset.
//!
//! **This writes flash and leaves an inert image on the part.** Reflash afterwards.
use std::time::Duration;

use probe_bench::{Attach, Bench, Verify};

/// Prefetch and cache control.
const CPUSS_CTL: u64 = 0x4040_1300;
/// Debug Exception and Monitor Control.
const DEMCR: u64 = 0xE000_EDFC;
/// `DEMCR.VC_CORERESET` — halt on a reset vector fetch.
const VC_CORERESET: u32 = 1 << 0;

fn main() -> anyhow::Result<()> {
    let elf = std::path::PathBuf::from(std::env::args().nth(1).expect("an inert ELF"));
    let address = match std::env::args().nth(2) {
        Some(text) => u64::from_str_radix(text.trim_start_matches("0x"), 16)?,
        None => panic!("pass a flash address to write, in hex"),
    };

    let attach = Attach {
        chip: "MSPM0L1306".to_owned(),
        probe: std::env::var("PROBE_RS_PROBE").ok(),
        // Whatever the part holds, it is about to hold something else.
        verify: Verify::Skip,
        ..Default::default()
    };
    let mut bench = Bench::attach(&attach, &elf)?;
    bench.program(&elf)?;
    println!("inert image on the part, CPUSS.CTL {:#010x}", bench.read_u32(CPUSS_CTL)?);

    let algorithm_left = dirty(&mut bench, address)?;
    println!("after a flash write       {algorithm_left:#010x}");
    std::thread::sleep(Duration::from_secs(3));
    let idle = bench.read_u32(CPUSS_CTL)?;
    println!("three seconds later       {idle:#010x}   <- control: nothing is resetting on its own");
    if idle != algorithm_left {
        println!("\nthe part changed it with no reset asked for. Nothing below can be read.");
        return Ok(());
    }

    // The CPU-only reset. `Core::reset` does not arm the catch, and an earlier `reset_and_halt` in
    // this session may have left it armed, so clear it rather than assuming.
    let demcr = bench.read_u32(DEMCR)?;
    bench.write_u32(DEMCR, demcr & !VC_CORERESET)?;
    {
        let mut core = bench.session().core(0)?;
        core.reset()?;
        core.halt(Duration::from_secs(1))?;
    }
    println!("\nCPU-only reset (SYSRESETREQ)  {:#010x}", bench.read_u32(CPUSS_CTL)?);

    // And the system reset, from the same starting state.
    let again = dirty(&mut bench, address)?;
    println!("back to the algorithm's state {again:#010x}");
    bench.reset_and_halt(Duration::from_secs(1))?;
    println!("system reset (VC_CORERESET)   {:#010x}", bench.read_u32(CPUSS_CTL)?);

    bench.leave_halted();
    println!("\nleft halted, holding the inert image. Reflash the real one.");
    Ok(())
}

/// Run the flash algorithm and leave `CPUSS.CTL` wherever its `Init` put it.
fn dirty(bench: &mut Bench, address: u64) -> Result<u32, probe_bench::Error> {
    let word = bench.read_u32(address)?;
    bench.write_flash(address, &word.to_le_bytes())?;
    bench.read_u32(CPUSS_CTL)
}
