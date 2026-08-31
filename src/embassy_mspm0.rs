//! Reading `embassy-mspm0`'s own exported state.
//!
//! Not about any particular firmware — this is the HAL's state, exported by the HAL under fixed
//! names for exactly this purpose. Any application built on it answers here.
//!
//! # Why it is worth a module
//!
//! **Sleep depth is the one thing an instrument cannot measure from outside**, because attaching a
//! probe holds the part awake. A current reading taken with a debugger on the pins says nothing
//! about what the firmware intended. These counts *are* the intent, and reading them is the
//! difference between "the board draws more than expected" and "something holds a guard at
//! `Standby1`".
//!
//! No halt and no volatile: the HAL keeps these as atomics, so the operations on them cannot be
//! folded away, and a byte cannot tear — five of them read consistently from a running core.

use crate::{Bench, Error};

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
    pub fn read(bench: &mut Bench) -> Result<Self, Error> {
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

        Ok(Self { blocks, min_sleep_ticks })
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

#[cfg(test)]
mod tests {
    use super::*;

    fn sleep(blocks: [u8; 5]) -> Sleep {
        Sleep { blocks, min_sleep_ticks: None }
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
