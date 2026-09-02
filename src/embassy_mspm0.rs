//! Reading `embassy-mspm0`'s own exported state.
//!
//! Not about any particular firmware — this is the HAL's state, exported by the HAL under fixed
//! names for exactly this purpose. Any application built on it answers here.
//!
//! # Why it is worth a module
//!
//! **A current reading taken with a debugger attached says nothing about sleep depth**, because the
//! debug power request stops the part reaching its deepest modes. These counts are the HAL's intent,
//! and reading them is the difference between "the board draws more than expected" and "something
//! holds a guard at `Standby1`".
//!
//! What the counts cannot say is what the part actually did. [`Entered`] reads that from the
//! hardware, and the two together are three different questions — see its docs.
//!
//! An earlier version of this paragraph said sleep depth cannot be observed from outside at all.
//! That is wrong, and wrong in the way that matters: the debug power request keeps the part out of
//! its deepest modes, but it does not stop the core reaching a `WFI`. `SCB.SCR.SLEEPDEEP` reads set
//! for as long as the core is parked, and probe-rs reports `CoreStatus::Sleeping` from
//! `DHCSR.S_SLEEP` for the same reason.
//!
//! No halt and no volatile: the HAL keeps these as atomics, so the operations on them cannot be
//! folded away, and a byte cannot tear — five of them read consistently from a running core.

use crate::{Error, Target};

/// One byte per level, in the HAL's own order.
pub const SLEEP_BLOCKS: &str = "embassy_mspm0_sleep_blocks";

/// The configured floor, below which a sleep is not worth entering.
///
/// Present only when the HAL is built with a time driver.
pub const MIN_SLEEP_TICKS: &str = "embassy_mspm0_min_sleep_ticks";

/// The sleep levels, shallowest first, matching `SleepLevel::LEVELS`.
pub const LEVELS: [&str; 5] = ["Stop0", "Stop1", "Stop2", "Standby0", "Standby1"];

/// What the HAL would do if the core went idle now.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Sleep {
    /// Guards held at each level, in [`LEVELS`] order.
    pub blocks: [u8; LEVELS.len()],
    /// The configured floor in ticks, or `None` when the image has no time driver.
    pub min_sleep_ticks: Option<u32>,
}

impl Sleep {
    /// Read it from a running core.
    pub fn read(bench: &mut impl Target) -> Result<Self, Error> {
        let bytes = bench.peek_bytes(SLEEP_BLOCKS, LEVELS.len())?;
        let mut blocks = [0u8; LEVELS.len()];
        blocks.copy_from_slice(&bytes);

        // Absent is not zero. Without a time driver there is no floor at all, and reporting one of
        // zero would say the opposite of what is true.
        let min_sleep_ticks = if bench.has(MIN_SLEEP_TICKS) {
            Some(bench.peek(MIN_SLEEP_TICKS)?)
        } else {
            None
        };

        Ok(Self {
            blocks,
            min_sleep_ticks,
        })
    }

    /// The deepest mode currently permitted, or `None` when all deep sleep is blocked.
    ///
    /// **Transcribed from the HAL's own `deepest_allowed`, not paraphrased**, because the rule is
    /// easy to state backwards: it is the *shallowest* guarded level that decides, and it caps the
    /// depth at the level above itself. A guard at the shallowest level of all leaves nothing but
    /// a plain `WFI`.
    #[must_use]
    pub fn deepest_allowed(&self) -> Option<&'static str> {
        for (i, &held) in self.blocks.iter().enumerate() {
            if held > 0 {
                return i.checked_sub(1).map(|j| LEVELS[j]);
            }
        }
        Some(LEVELS[LEVELS.len() - 1])
    }

    /// The levels holding a guard, shallowest first.
    pub fn held(&self) -> impl Iterator<Item = (&'static str, u8)> + '_ {
        LEVELS
            .iter()
            .zip(self.blocks)
            .filter(|(_, held)| *held > 0)
            .map(|(name, held)| (*name, held))
    }
}

/// Where the hardware records what a deep sleep did, on `mspm0l130x`.
///
/// Addresses cross-checked against `mspm0-data` and TI's `hw_sysctl` headers, which agree.
mod reg {
    /// `SCB.SCR`, and the bit that arms a deep sleep. Architectural rather than device data.
    pub const SCR: u64 = 0xE000_ED10;
    pub const SLEEPDEEP: u32 = 1 << 2;

    // `DHCSR.S_SLEEP` is `0xE000_EDF0` bit 17 and is deliberately **not** listed here, because
    // reading it as a memory word does not work and looks as though it does. See [`Entered::read`].

    /// `SYSCTL.PMODECFG`, whose `DSLEEP` field picks the family.
    pub const PMODECFG: u64 = 0x400B_0140;
    pub const DSLEEP: u32 = 0x3;
    pub const DSLEEP_STOP: u32 = 0;
    pub const DSLEEP_STANDBY: u32 = 1;
    pub const DSLEEP_SHUTDOWN: u32 = 2;

    /// `SYSCTL.SYSOSCCFG`, which picks between STOP0, STOP1 and STOP2.
    pub const SYSOSCCFG: u64 = 0x400B_0100;
    pub const USE4MHZSTOP: u32 = 1 << 8;
    pub const DISABLESTOP: u32 = 1 << 9;

    /// `SYSCTL.MCLKCFG`, whose `STOPCLKSTBY` picks between STANDBY0 and STANDBY1.
    pub const MCLKCFG: u64 = 0x400B_0104;
    pub const STOPCLKSTBY: u32 = 1 << 21;
}

/// What the hardware says about sleeping, as against what the HAL would allow.
///
/// **Three questions, and they are not the same one.** [`Sleep`] answers what is permitted right
/// now. This answers what is happening now and what last happened. A pane that shows one of the
/// three and calls it "the sleep level" is answering a question nobody asked.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Entered {
    /// Whether the core is parked at a `WFI` right now.
    ///
    /// **Must come from probe-rs's `CoreStatus`, never from a read of `DHCSR`** — see
    /// [`Entered::read`], which takes it as an argument for that reason.
    pub parked: bool,
    /// `SCB.SCR.SLEEPDEEP`: the sleep the core is in, or about to enter, is a deep one.
    ///
    /// The HAL sets this immediately before `WFI` and clears it immediately after, so it is held for
    /// as long as the core is parked — measured set in 2495 of 2498 samples on a mostly-idle part.
    ///
    /// **On its own it does not say the core is asleep**, only that deep sleep is armed. The two
    /// instructions between setting it and the `WFI` are a window where it reads set on a running
    /// core. Pair it with [`Entered::parked`], which is what [`Entered::now`] does.
    pub deep_armed: bool,
    /// The mode most recently entered, decoded from SYSCTL, or `None` before any deep sleep.
    pub last: Option<&'static str>,
    /// The three registers as read, for a pane that wants to show the raw values.
    pub pmodecfg: u32,
    pub sysosccfg: u32,
    pub mclkcfg: u32,
}

impl Entered {
    /// Read it from a running core.
    ///
    /// `parked` must come from probe-rs's `CoreStatus` — `matches!(bench.status()?,
    /// CoreStatus::Sleeping)`. It is an argument rather than a read here because there is no way to
    /// obtain it correctly from a [`Target`], and the way that looks correct is wrong:
    ///
    /// **A memory read of `DHCSR` reports the core awake however soundly it is sleeping.** The bit
    /// is real and probe-rs reports it, but reaching `0xE000_EDF0` as a memory word is an AHB-AP
    /// access to the PPB, and servicing that bus request wakes the core far enough to deassert
    /// `S_SLEEP` before the read samples it. Measured on one board: probe-rs said `Sleeping` in 374
    /// of 375 polls while a raw read of the same register said awake in 357 of 357, in the same
    /// session, seconds apart.
    ///
    /// `SCB.SCR` is on the same bus and does **not** have this problem, and the difference is the
    /// point: `SCR` holds configuration, which a transient wake does not change, where `S_SLEEP`
    /// describes the core, which is exactly what the transient wake changes.
    ///
    /// # Errors
    ///
    /// If any of the four reads fails.
    pub fn read(bench: &mut impl Target, parked: bool) -> Result<Self, Error> {
        let pmodecfg = bench.read_u32(reg::PMODECFG)?;
        let sysosccfg = bench.read_u32(reg::SYSOSCCFG)?;
        let mclkcfg = bench.read_u32(reg::MCLKCFG)?;
        let scr = bench.read_u32(reg::SCR)?;

        Ok(Self {
            parked,
            deep_armed: scr & reg::SLEEPDEEP != 0,
            last: decode(pmodecfg, sysosccfg, mclkcfg),
            pmodecfg,
            sysosccfg,
            mclkcfg,
        })
    }

    /// What the core is doing at this instant, from the two architectural bits together.
    ///
    /// Neither answers it alone: the core status cannot tell a deep sleep from a plain `WFI`, and
    /// `SLEEPDEEP` is armed for a couple of instructions before the core actually parks.
    #[must_use]
    pub fn now(&self) -> &'static str {
        match (self.parked, self.deep_armed) {
            (true, true) => "parked in a deep sleep",
            (true, false) => "parked in a plain WFI",
            // Deep sleep armed but not yet parked: the handful of instructions between setting the
            // bit and the `WFI`. Catching one is rare and is not a fault.
            (false, true) => "running, with deep sleep armed",
            (false, false) => "running",
        }
    }
}

/// Which level the three registers describe.
///
/// **`DSLEEP` is read first and decides which other register is meaningful.** Each sub-mode register
/// is written only on its own kind of entry, so after a STANDBY the STOP selectors still hold
/// whatever the last STOP left — a plausible sub-mode with nothing marking it stale. Consulting both
/// and picking the one that looks set would report that residue about half the time.
fn decode(pmodecfg: u32, sysosccfg: u32, mclkcfg: u32) -> Option<&'static str> {
    match pmodecfg & reg::DSLEEP {
        reg::DSLEEP_STOP => Some(
            match (sysosccfg & reg::USE4MHZSTOP != 0, sysosccfg & reg::DISABLESTOP != 0) {
                (false, false) => "Stop0",
                (true, false) => "Stop1",
                (false, true) => "Stop2",
                // Both policies at once is not a level the HAL selects. Reporting the nearest one
                // would invent a reading; naming it is what lets somebody chase it.
                (true, true) => "Stop?, both SYSOSC policies set",
            },
        ),
        reg::DSLEEP_STANDBY => Some(if mclkcfg & reg::STOPCLKSTBY != 0 {
            "Standby1"
        } else {
            "Standby0"
        }),
        reg::DSLEEP_SHUTDOWN => Some("Shutdown"),
        // `DSLEEP` is two bits with three values, so this is genuinely reserved rather than a level
        // this crate has not heard of.
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sleep(blocks: [u8; 5]) -> Sleep {
        Sleep {
            blocks,
            min_sleep_ticks: None,
        }
    }

    /// Nothing held is the deepest mode the part has.
    #[test]
    fn no_guards_allows_the_deepest_level() {
        assert_eq!(sleep([0; 5]).deepest_allowed(), Some("Standby1"));
        assert_eq!(sleep([0; 5]).held().count(), 0);
    }

    /// A guard at the shallowest level leaves no deep sleep at all, which is the case a `None`
    /// return exists for and the one an off-by-one would turn into `Stop0`.
    #[test]
    fn a_guard_at_the_shallowest_level_blocks_everything() {
        assert_eq!(sleep([1, 0, 0, 0, 0]).deepest_allowed(), None);
    }

    /// **The shallowest guard decides, not the deepest.** Stated as a case where the two differ,
    /// because a rule read backwards agrees with one read forwards whenever only one is held.
    #[test]
    fn the_shallowest_guard_is_the_one_that_caps() {
        let both = sleep([0, 1, 0, 0, 1]);
        assert_eq!(both.deepest_allowed(), Some("Stop0"));
        assert_ne!(both.deepest_allowed(), Some("Standby0"));

        assert_eq!(sleep([0, 0, 1, 0, 0]).deepest_allowed(), Some("Stop1"));
        assert_eq!(sleep([0, 0, 0, 0, 1]).deepest_allowed(), Some("Standby0"));
    }

    /// A count above one is several holders, and it still caps at the same place.
    #[test]
    fn a_count_is_holders_not_depth() {
        assert_eq!(sleep([0, 0, 7, 0, 0]).deepest_allowed(), Some("Stop1"));
        let held: Vec<_> = sleep([0, 0, 7, 0, 2]).held().collect();
        assert_eq!(held, vec![("Stop2", 7), ("Standby1", 2)]);
    }
}

#[cfg(test)]
mod entered_tests {
    use super::*;

    /// `DSLEEP` in the low bits, with the rest of the word set, so a decode that masked wrongly
    /// would read a different level rather than the same one.
    fn pmodecfg(dsleep: u32) -> u32 {
        0xDEAD_BE00 | dsleep
    }

    #[test]
    fn each_stop_sub_mode_decodes() {
        assert_eq!(decode(pmodecfg(0), 0, 0), Some("Stop0"));
        assert_eq!(decode(pmodecfg(0), reg::USE4MHZSTOP, 0), Some("Stop1"));
        assert_eq!(decode(pmodecfg(0), reg::DISABLESTOP, 0), Some("Stop2"));
    }

    #[test]
    fn each_standby_sub_mode_decodes() {
        assert_eq!(decode(pmodecfg(1), 0, 0), Some("Standby0"));
        assert_eq!(decode(pmodecfg(1), 0, reg::STOPCLKSTBY), Some("Standby1"));
        assert_eq!(decode(pmodecfg(2), 0, 0), Some("Shutdown"));
    }

    /// **The trap this decode exists to avoid.** Each sub-mode register is written only on its own
    /// kind of entry, so a part that last entered STANDBY still carries whatever the previous STOP
    /// left in `SYSOSCCFG`. Both words say "Stop1" here, and the answer must be a standby anyway.
    ///
    /// This is the case that was observed on a board: `DSLEEP` STANDBY with `SYSOSCCFG` holding a
    /// STOP pattern from earlier.
    #[test]
    fn a_stale_stop_selector_does_not_leak_into_a_standby() {
        let stale = reg::USE4MHZSTOP;
        assert_eq!(decode(pmodecfg(1), stale, reg::STOPCLKSTBY), Some("Standby1"));
        assert_eq!(decode(pmodecfg(1), stale, 0), Some("Standby0"));

        // And the mirror: a standby selector left set must not reach a stop decode.
        assert_eq!(decode(pmodecfg(0), 0, reg::STOPCLKSTBY), Some("Stop0"));
    }

    /// `DSLEEP` is two bits with three values, so the fourth is reserved rather than a level this
    /// crate has not heard of. Reporting it as a level would invent one.
    #[test]
    fn the_reserved_dsleep_value_is_not_a_level() {
        assert_eq!(decode(pmodecfg(3), 0, 0), None);
    }

    /// Both SYSOSC policies at once is not a combination the HAL writes. Naming it beats picking
    /// whichever level is nearest, which would read as a normal answer.
    #[test]
    fn an_impossible_stop_pattern_says_so() {
        let both = decode(pmodecfg(0), reg::USE4MHZSTOP | reg::DISABLESTOP, 0).unwrap();
        assert!(both.contains("both"), "{both}");
        assert!(
            !LEVELS.contains(&both),
            "an impossible pattern named a real level: {both}"
        );
    }
}

#[cfg(test)]
mod now_tests {
    use super::*;

    fn at(parked: bool, deep_armed: bool) -> Entered {
        Entered {
            parked,
            deep_armed,
            last: None,
            pmodecfg: 0,
            sysosccfg: 0,
            mclkcfg: 0,
        }
    }

    /// **Neither bit answers it alone, which is why both are read.** Stated as the two cases where
    /// one bit is set and the other is not — a reading that consulted only one would call these the
    /// same as a neighbour, and each is a different thing to see on a board.
    #[test]
    fn one_bit_cannot_tell_these_apart() {
        assert_eq!(at(true, true).now(), "parked in a deep sleep");
        assert_eq!(at(true, false).now(), "parked in a plain WFI");
        assert_eq!(at(false, true).now(), "running, with deep sleep armed");
        assert_eq!(at(false, false).now(), "running");

        // `SLEEPDEEP` alone would merge these two, and it is the merge that would read as a part
        // sleeping deeply when it is awake.
        assert_ne!(at(true, true).now(), at(false, true).now());
        // `S_SLEEP` alone would merge these two, which is the whole point of the sub-mode work.
        assert_ne!(at(true, true).now(), at(true, false).now());
    }
}
