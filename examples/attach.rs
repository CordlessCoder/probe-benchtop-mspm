//! Attach to a running board, say what it is doing, and resolve a symbol.
//!
//! The smoke test for the crate: if this cannot report a plausible status and find a symbol, no
//! sweep built on it will work either.
//!
//! ```text
//! cargo run --example attach -- <chip> <elf> [symbol]...
//! PROBE=0451:bef3-5:ML130001 cargo run --example attach -- MSPM0L1306 firmware.elf
//! ```

use std::path::PathBuf;

use probe_bench::{Attach, Bench, Target};

fn main() {
    // `Display`, not `Debug`. Every message in this crate is written to be read, and returning the
    // error from `main` would print the struct instead.
    if let Err(e) = run() {
        eprintln!("attach: {e}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let chip = args.next().ok_or("usage: attach <chip> <elf> [symbol]...")?;
    let elf = PathBuf::from(args.next().ok_or("usage: attach <chip> <elf> [symbol]...")?);
    let wanted: Vec<String> = args.collect();

    let attach = Attach {
        chip,
        probe: std::env::var("PROBE").ok(),
        ..Attach::default()
    };

    let mut bench = Bench::attach(&attach, &elf)?;
    println!("attached, {} symbols in {}", bench.symbols().len(), elf.display());
    println!("status: {:?}, live: {}", bench.status()?, bench.is_live()?);
    if std::env::var("RESUME").is_ok() {
        bench.resume()?;
        println!("after resume: {:?}, live: {}", bench.status()?, bench.is_live()?);
    }

    for name in &wanted {
        match bench.symbols().get(name) {
            Ok(symbol) => {
                print!("{name}: {:#010x}, {} B", symbol.address, symbol.size);
                match bench.peek::<u32>(name) {
                    Ok(value) => println!(" = {value} ({value:#x})"),
                    Err(e) => println!("  [{e}]"),
                }
            }
            Err(e) => println!("{e}"),
        }
    }

    Ok(())
}
