//! Who puts `CPUSS.CTL` back after probe-rs flashes an MSPM0?
//!
//! The algorithm in `MSPM0L_Series.yaml` clears the prefetch and cache bits in `Init` and its
//! `UnInit` is `movs r0, #0; bx lr` — so nothing in the flashing path restores them. probe-rs's own
//! `reset_system` comment says an application flashed this way runs slowed down until the next
//! power cycle.
//!
//! This reads the register at four moments and says whether that is true of the part in front of
//! it: after a flash operation, at the halt of the reset that follows, and after the application
//! has run. **An application that configures the prefetcher itself repairs this at `init` and would
//! hide it**, which is the case the third and fourth readings separate.
use std::time::Duration;

use probe_bench::{Attach, Bench, Verify};

/// Prefetch and cache control.
const CPUSS_CTL: u64 = 0x4040_1300;

fn main() -> anyhow::Result<()> {
    let elf = std::path::PathBuf::from(std::env::args().nth(1).expect("an ELF"));
    let attach = Attach {
        chip: "MSPM0L1306".to_owned(),
        probe: std::env::var("PROBE_RS_PROBE").ok(),
        verify: Verify::Sampled,
        ..Default::default()
    };
    let mut bench = Bench::attach(&attach, &elf)?;

    println!("as found                {:#010x}", bench.read_u32(CPUSS_CTL)?);

    bench.program(&elf)?;
    println!("after flashing          {:#010x}", bench.read_u32(CPUSS_CTL)?);

    bench.reset_and_halt(Duration::from_secs(1))?;
    println!("at the reset vector     {:#010x}", bench.read_u32(CPUSS_CTL)?);

    bench.resume()?;
    std::thread::sleep(Duration::from_millis(300));
    println!("once the image has run  {:#010x}", bench.read_u32(CPUSS_CTL)?);
    Ok(())
}
