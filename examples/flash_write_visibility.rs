//! Is a flash write made through the debug port visible to the CPU that runs next?
//!
//! **On an MSPM0L1306 it is, on the first reset.** This exists because the opposite was believed for
//! a day, on the strength of a symbol that is published once at boot and was read without ever
//! rebooting the part. One reset settles it, and this is the reset.
//!
//! # What it does
//!
//! Takes a flash address the application reads at boot, and the name of a RAM symbol it publishes
//! that read to. Then, in one attach:
//!
//! 1. Resets, and reads the symbol and the flash word. Both should agree, or nothing below means
//!    anything and it says so and stops.
//! 2. Rewrites the flash word through [`Bench::write_flash`], and reads it back over the probe.
//! 3. Resets again, and reads the symbol. **A CPU reporting the old word here would be the bug.**
//! 4. Only if it does: clears and restores `CPUSS.CTL`'s three cache bits, resets, and reads once
//!    more. Fresh would say the caches held the stale copy and clearing the bit invalidates them.
//! 5. Puts the original word back and resets, so the part is left as it was found.
//!
//! `CPUSS.CTL` is printed along the way, because what a flash operation leaves it at is a separate
//! question with a separate answer — see `cpuss_ctl_after_flash`.
//!
//! # Why it is shaped this way
//!
//! The probe's own reads go through the debug access port and see flash directly, so they cannot
//! answer this — only code running on the core can. The application has to supply that, which is
//! why the address and the symbol are arguments rather than constants.
//!
//! **This writes flash.** The word is put back at the end, and the rest of the sector is preserved
//! by `keep_unwritten_bytes`, but a run that dies in the middle leaves one word changed.
use std::time::Duration;

use probe_bench::{Attach, Bench, Verify};

/// Prefetch and cache control.
const CPUSS_CTL: u64 = 0x4040_1300;
/// `CPUSS.CTL` bits `LITEN`, `ICACHE` and `PREFETCH`.
const CACHES: u32 = 0b111;

/// Long enough for the application to reach whatever publishes the symbol.
const BOOT: Duration = Duration::from_millis(300);

fn main() -> anyhow::Result<()> {
    let mut args = std::env::args().skip(1);
    let elf = std::path::PathBuf::from(args.next().expect("an ELF"));
    let address = {
        let text = args.next().expect("a flash address to write, in hex");
        u64::from_str_radix(text.trim_start_matches("0x"), 16)?
    };
    let symbol = args.next().expect("the symbol the application publishes it to");

    let attach = Attach {
        chip: "MSPM0L1306".to_owned(),
        probe: std::env::var("PROBE_RS_PROBE").ok(),
        verify: Verify::Sampled,
        ..Default::default()
    };
    let mut bench = Bench::attach(&attach, &elf)?;

    let original = bench.read_u32(address)?;
    let baseline = boot(&mut bench, &symbol)?;
    println!("as found          flash {original:#010x}  {symbol} {baseline:#010x}");
    if baseline != original {
        println!("  the symbol does not report this word; nothing below means anything");
        return Ok(());
    }

    // Any different word will do, and the sector is erased and rewritten so no bit direction is
    // out of reach.
    let written = original ^ 0x0000_ff00;
    bench.write_flash(address, &written.to_le_bytes())?;
    println!("after the write   flash {:#010x}  CPUSS.CTL {:#010x}", bench.read_u32(address)?, bench.read_u32(CPUSS_CTL)?);

    let after_reset = boot(&mut bench, &symbol)?;
    println!("after a reset     {symbol} {after_reset:#010x}  CPUSS.CTL {:#010x}", bench.read_u32(CPUSS_CTL)?);
    if after_reset == written {
        println!("\nthe CPU sees the write on the first reset. Nothing to explain.");
        return restore(&mut bench, address, original, &symbol);
    }
    println!("\nthe CPU reports the word from before the write.");

    let ctl = bench.read_u32(CPUSS_CTL)?;
    bench.write_u32(CPUSS_CTL, ctl & !CACHES)?;
    println!("caches off        CPUSS.CTL {:#010x}", bench.read_u32(CPUSS_CTL)?);
    bench.write_u32(CPUSS_CTL, ctl)?;
    println!("caches back       CPUSS.CTL {:#010x}", bench.read_u32(CPUSS_CTL)?);

    let after_toggle = boot(&mut bench, &symbol)?;
    println!("after a reset     {symbol} {after_toggle:#010x}");
    println!(
        "\n{}",
        if after_toggle == written {
            "clearing the bit invalidates: an invalidate after the last program is the fix."
        } else {
            "clearing the bit does not invalidate: restoring CPUSS.CTL would not fix this."
        }
    );

    restore(&mut bench, address, original, &symbol)
}

/// Reset, let the application run, and read what it published.
fn boot(bench: &mut Bench, symbol: &str) -> Result<u32, probe_bench::Error> {
    bench.reset_and_halt(Duration::from_secs(1))?;
    bench.resume()?;
    std::thread::sleep(BOOT);
    bench.peek::<u32>(symbol)
}

fn restore(bench: &mut Bench, address: u64, original: u32, symbol: &str) -> anyhow::Result<()> {
    bench.write_flash(address, &original.to_le_bytes())?;
    let back = boot(bench, symbol)?;
    println!("\nput back          flash {:#010x}  {symbol} {back:#010x}", bench.read_u32(address)?);
    Ok(())
}
