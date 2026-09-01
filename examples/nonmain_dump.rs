//! Dump an MSPM0's NONMAIN boot configuration, and the first words of MAIN beside it.
//!
//! NONMAIN holds the boot configuration: the SWD port gate, the debug access policy, the mass erase
//! and factory reset gates, their passwords, and a CRC32 over each structure. Nothing in probe-rs
//! reads it back, and it is the region whose contents decide whether a part can be recovered at all.
//!
//! Reading it is free and answers two questions a recovery attempt should not have to guess at:
//! whether the part is at TI's defaults, and whether the factory reset gate is armed.
//!
//! MAIN's first words are printed alongside because the two regions are what a DSSM mass erase
//! separates: it clears MAIN and leaves NONMAIN, and a dump either side of one shows that.

use probe_bench::{Attach, Bench, Verify};

/// Where the boot configuration lives.
const NONMAIN: u64 = 0x41C0_0000;
/// Where the application's vector table lives.
const MAIN: u64 = 0x0000_0000;

fn main() -> anyhow::Result<()> {
    let attach = Attach {
        chip: "MSPM0L1306".to_owned(),
        probe: std::env::var("PROBE_RS_PROBE").ok(),
        // The image on the part is not the point here, and after a mass erase there is none.
        verify: Verify::Skip,
        ..Default::default()
    };
    let elf = std::path::PathBuf::from(std::env::args().nth(1).expect("an ELF to attach with"));
    let mut bench = Bench::attach(&attach, &elf)?;

    println!("MAIN, first four words:");
    dump(&mut bench, MAIN, 1);

    println!("\nNONMAIN:");
    dump(&mut bench, NONMAIN, 6);

    println!("\nBCR fields that decide recoverability:");
    let mut word = |at: u64| bench.read_u32(NONMAIN + at).unwrap_or(0);
    let halves = |w: u32| (w & 0xFFFF, w >> 16);

    let (debug_access, swdp) = halves(word(0x04));
    let (mass_erase, factory_reset) = halves(word(0x20));
    for (name, value) in [
        ("debugAccess", debug_access),
        ("swdpMode", swdp),
        ("massEraseMode", mass_erase),
        ("factoryResetMode", factory_reset),
    ] {
        // AABB is enabled, CCDD enabled-with-password, and every other value is disabled.
        let meaning = match value {
            0xAABB => "enabled",
            0xCCDD => "enabled, password required",
            _ => "DISABLED",
        };
        println!("  {name:<18} {value:#06x}  {meaning}");
    }
    println!("  {:<18} {:#010x}", "userCfgCRC", word(0x5C));
    Ok(())
}

fn dump(bench: &mut Bench, base: u64, rows: u64) {
    for row in 0..rows {
        let at = base + row * 16;
        let words: Vec<String> = (0..4)
            .map(|i| {
                bench
                    .read_u32(at + i * 4)
                    .map_or_else(|_| "--------".to_owned(), |v| format!("{v:08x}"))
            })
            .collect();
        println!("  {at:08x}  {}", words.join(" "));
    }
}
