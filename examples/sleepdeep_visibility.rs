//! Can a debugger see `SCB.SCR.SLEEPDEEP`, or is the bit set only where it cannot be read?
//!
//! # The claim being tested
//!
//! A HAL that sleeps by setting `SLEEPDEEP`, executing `WFI`, and clearing it again holds the bit
//! set for exactly as long as the core is parked. So the reasoning goes: a probe holds the part
//! awake, therefore the bit is never set while anything is looking, therefore a tool that reads
//! `SCR` reports zero forever and looks like it is working.
//!
//! **Two different things are being run together there, and only one of them is true.** The debug
//! power request does keep the debug domain up, and on this family that stops the part reaching its
//! deepest modes — which is why a quiescent-current figure taken with a probe attached is worthless.
//! It does not follow that the core never reaches the `WFI`. `WFI` stops the core until a wake
//! event whatever the debug domain is doing, and reading a PPB register is a bus access through the
//! AHB-AP that does not need the core to be running.
//!
//! So the bit may well be observable. This settles it by looking.
//!
//! # What it does
//!
//! Samples `SCR` as fast as the link allows and counts how often `SLEEPDEEP` reads back set. It
//! also reports the three SYSCTL registers that persist, as a control: those are written on the way
//! into a deep sleep and nothing clears them, so a part that has slept at all shows it there. If
//! `SLEEPDEEP` never reads set *and* SYSCTL says the part has been sleeping, the claim holds. If
//! `SLEEPDEEP` reads set even once, it does not.
//!
//! **A sampled miss is weak evidence and a hit is strong evidence.** Never seeing the bit could mean
//! it is unobservable, or that the sampling never landed in the window — so the run reports the
//! sample count and leaves that distinction to the reader rather than concluding from a zero.
//!
//! ```text
//! cargo run --example sleepdeep_visibility -- <elf>
//! ```

use std::time::Instant;

use probe_bench::{Attach, Bench, Target};

/// `SCB.SCR`, and the bit that arms deep sleep. Architectural, so this is not device data.
const SCR: u64 = 0xE000_ED10;
const SLEEPDEEP: u32 = 1 << 2;

/// SYSCTL on `mspm0l130x`, from the device data and TI's own headers, which agree.
const SYSOSCCFG: u64 = 0x400B_0100;
const MCLKCFG: u64 = 0x400B_0104;
const PMODECFG: u64 = 0x400B_0140;

fn main() -> anyhow::Result<()> {
    let elf = std::env::args().nth(1).ok_or_else(|| anyhow::anyhow!("usage: <elf>"))?;
    let attach = Attach {
        chip: std::env::var("BENCH_CHIP").unwrap_or_else(|_| "MSPM0L1306".to_owned()),
        probe: std::env::var("PROBE_RS_PROBE").ok(),
        ..Attach::default()
    };
    let mut bench = Bench::attach(&attach, std::path::Path::new(&elf))?;
    // Reset rather than take the part as found: attaching can leave it halted, and a halted core
    // never reaches a `WFI` at all. This makes the run repeatable from any starting state.
    bench.reset()?;
    std::thread::sleep(std::time::Duration::from_millis(500));

    // **`Sleeping` is the state this is about, so it is not an error.** probe-rs reports it from
    // `DHCSR.S_SLEEP`, which is itself the first piece of evidence: the core is parked at a `WFI`
    // with a probe attached, which is what the claim under test says cannot be happening.
    let status = bench.status()?;
    println!("core status after reset: {status:?}");
    anyhow::ensure!(!status.is_halted(), "the core is halted, so it never reaches a WFI");

    const DHCSR: u64 = 0xE000_EDF0;
    const S_SLEEP: u32 = 1 << 17;

    // One acquisition for the whole sweep: the question is what the core is doing, and taking the
    // core per sample would spend most of the run in the handshake rather than looking.
    //
    // **Two orders, because reading may perturb.** A bus access to SYSCTL touches a peripheral the
    // part powers down in a deep sleep, where the two architectural bits are on the PPB and do not.
    // If the SYSCTL reads wake the core, the second column drops and the first does not — which no
    // single-order sweep could tell from a quiet part.
    let started = Instant::now();
    let (mut deep, mut parked, mut total) = (0u32, 0u32, 0u32);
    let (mut deep_after, mut parked_after, mut after_total) = (0u32, 0u32, 0u32);
    let mut held = bench.hold()?;
    while started.elapsed().as_secs() < 5 {
        // Bare: the two PPB words and nothing else.
        let scr = held.read_u32(SCR)?;
        let dhcsr = held.read_u32(DHCSR)?;
        total += 1;
        deep += u32::from(scr & SLEEPDEEP != 0);
        parked += u32::from(dhcsr & S_SLEEP != 0);

        // After touching SYSCTL, which is what the real reader does.
        let _ = held.read_u32(PMODECFG)?;
        let _ = held.read_u32(SYSOSCCFG)?;
        let _ = held.read_u32(MCLKCFG)?;
        let scr = held.read_u32(SCR)?;
        let dhcsr = held.read_u32(DHCSR)?;
        after_total += 1;
        deep_after += u32::from(scr & SLEEPDEEP != 0);
        parked_after += u32::from(dhcsr & S_SLEEP != 0);
    }
    drop(held);
    let (set, total) = (deep, total);

    // **probe-rs's own status, for the same question.** It reports `Sleeping` from `S_SLEEP`, so if
    // a raw read of `DHCSR` says one thing and this says another, the difference is in how the
    // register is reached rather than in what the core is doing.
    let mut by_status = 0u32;
    let mut status_total = 0u32;
    let started = Instant::now();
    while started.elapsed().as_secs() < 3 {
        if matches!(bench.status()?, probe_bench::CoreStatus::Sleeping) {
            by_status += 1;
        }
        status_total += 1;
    }
    println!("probe-rs CoreStatus::Sleeping in {by_status} of {status_total}");

    let pmodecfg = bench.read_u32(PMODECFG)?;
    let sysosccfg = bench.read_u32(SYSOSCCFG)?;
    let mclkcfg = bench.read_u32(MCLKCFG)?;

    println!("read bare (PPB only):");
    println!("  SCR.SLEEPDEEP  set in {set} of {total}");
    println!("  DHCSR.S_SLEEP  set in {parked} of {total}");
    println!("after reading the three SYSCTL words first:");
    println!("  SCR.SLEEPDEEP  set in {deep_after} of {after_total}");
    println!("  DHCSR.S_SLEEP  set in {parked_after} of {after_total}");
    println!();
    println!(
        "PMODECFG  {pmodecfg:#010x}  DSLEEP={}",
        match pmodecfg & 0x3 {
            0 => "STOP",
            1 => "STANDBY",
            2 => "SHUTDOWN",
            _ => "reserved",
        }
    );
    println!(
        "SYSOSCCFG {sysosccfg:#010x}  USE4MHZSTOP={} DISABLESTOP={}",
        sysosccfg >> 8 & 1,
        sysosccfg >> 9 & 1
    );
    println!("MCLKCFG   {mclkcfg:#010x}  STOPCLKSTBY={}", mclkcfg >> 21 & 1);
    println!();
    if set > 0 {
        println!("The bit is observable. A pane may read it.");
    } else {
        println!("Never seen set. Weak on its own — check SYSCTL above says the part has slept.");
    }
    Ok(())
}
