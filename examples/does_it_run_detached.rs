//! Does the part keep running when the debugger lets go?
//!
//! **The question any current measurement turns on.** A quiescent-current measurement is worthless with
//! a probe attached, because the debug power request holds the part awake — so the only way a
//! debug-driven harness can serve that case is to set the part going, detach, and measure. Whether
//! that is possible is a property of what a detach does, and it has been recorded here as an
//! observation without a mechanism: a counter the application advances "reads zero immediately after
//! a tool that resumed the core exits, and zero again twelve seconds later".
//!
//! Zero is the interesting part. A part that merely *stopped* would hold its last count; a part
//! reading zero has had its RAM cleared, which is a reset rather than a halt. This separates the two,
//! and says which side does it.
//!
//! # How it distinguishes them
//!
//! One process, three phases, and no other tool running in between:
//!
//! 1. Attach, reset, and watch the counter climb — so the rate is known rather than assumed.
//! 2. **Drop the session** and sleep, with nothing attached at all.
//! 3. Attach again and read.
//!
//! If the counter continues from where phase 1 left it, the part ran while detached. If it reads
//! zero, the part's RAM was cleared, which is a reset.
//!
//! # What it cannot show, and this is the limit rather than a gap in the experiment
//!
//! **A reset seen in phase 3 does not say when it happened.** The only way to read the counter is to
//! attach, and attaching is one of the two candidates for having caused it — so a part that ran
//! happily for the whole detached window and reset the moment it was looked at is indistinguishable
//! from one that reset at the drop and sat there.
//!
//! Answering *that* needs state a reset does not clear: a counter in `.uninit`, which the startup
//! code does not zero, or something written to flash. Neither exists in the image this runs against.
//! Measured 2026-09-01: phase 1 climbs at the rate the application advances it, phase 3 reads zero.
use std::time::{Duration, Instant};

use probe_bench::{Attach, Bench, Verify};

/// A counter the application advances on its own, at whatever rate it chooses.
const COUNTER: &str = "a_counter";

fn main() -> anyhow::Result<()> {
    let mut args = std::env::args().skip(1);
    let elf = std::path::PathBuf::from(args.next().expect("an ELF"));
    let detached: u64 = args.next().map_or(Ok(20), |v| v.parse())?;

    let attach = Attach {
        chip: "MSPM0L1306".to_owned(),
        probe: std::env::var("PROBE_RS_PROBE").ok(),
        verify: Verify::Sampled,
        ..Default::default()
    };

    let watched = {
        let mut bench = Bench::attach(&attach, &elf)?;
        bench.reset()?;
        println!("phase 1: attached, reset, watching for {detached}s");
        let started = Instant::now();
        let mut last = 0;
        while started.elapsed() < Duration::from_secs(detached) {
            std::thread::sleep(Duration::from_secs(2));
            last = bench.peek::<u32>(COUNTER)?;
            println!("  {:>4.0}s  {COUNTER} = {last}", started.elapsed().as_secs_f64());
        }
        last
    };
    // The session is dropped here, which is the whole of the experiment.

    println!("\nphase 2: detached for {detached}s, nothing attached");
    std::thread::sleep(Duration::from_secs(detached));

    let mut bench = Bench::attach(&attach, &elf)?;
    // **Read before resuming.** Attaching can leave the core halted, and resuming first would let it
    // take a pass between the attach and the read — which is a count this experiment did not observe
    // being made.
    let after = bench.peek::<u32>(COUNTER)?;
    bench.resume()?;
    println!("\nphase 3: attached again, {COUNTER} = {after}");

    println!();
    if after >= watched {
        println!("It kept counting: {watched} before the detach, {after} after.");
        println!("The part runs while detached, and a current measurement is reachable.");
    } else {
        println!("It went backwards: {watched} before the detach, {after} after.");
        println!("The part did not survive the detach. Whether that was the drop or the attach is");
        println!("the next question, and phase 3's climb against the clock answers it.");
    }

    // Resumed above, so this says whether the part is alive at all after a re-attach — separately
    // from whether it survived the detach.
    println!("\nand with the core resumed:");
    let started = Instant::now();
    for _ in 0..3 {
        std::thread::sleep(Duration::from_secs(2));
        println!(
            "  {:>4.0}s  {COUNTER} = {}",
            started.elapsed().as_secs_f64(),
            bench.peek::<u32>(COUNTER)?
        );
    }
    Ok(())
}
