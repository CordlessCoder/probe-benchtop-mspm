//! The MSPM0's flash controller, driven from the host with the core halted.
//!
//! [`Bench::write_flash`](crate::Bench::write_flash) runs a flash algorithm on the core, so it can
//! only rebuild whole sectors, and it resets the part afterwards because the algorithm overwrote
//! RAM. This drives `FLASHCTL` directly over the debug port instead: blank-verify one flash word,
//! program one, erase one sector. Nothing runs on the core and RAM is untouched.
//!
//! The command sequence is the HAL's, register for register. What varies across the family is
//! which protection register covers a sector, and whether the flash has ECC; both come from a
//! per-part table generated from the same catalog the HAL reads. A part the table does not know is
//! refused.
//!
//! # The core stays halted while a [`Flash`] exists
//!
//! The controller takes the bank away from the CPU for each command, and on parts carrying
//! `FLASH_ERR_06` a fetch during one returns wrong data rather than stalling. [`Flash::new`] halts
//! the core and dropping it lets the core run again, if it was running before.
//!
//! **Two things halting does not cover.** A DMA channel reading flash keeps running. And a
//! firmware halted inside its own flash command resumes to find this module's status in `STATCMD`,
//! not its own.
//!
//! # What the controller refuses
//!
//! `CMDCTL.DATAVEREN` is set, as the HAL sets it, so programming a word in a way that needs a
//! stored zero to return to one is refused with [`Fault::NotErased`] instead of silently doing
//! nothing. Blank-verify answers `true` for a word programmed all-ones: it cannot tell the two
//! apart.
//!
//! NONMAIN is out of reach: every address is checked against MAIN's size, as the part reports it.

use std::time::{Duration, Instant};

use crate::{CoreStatus, Error, Held, Target};

/// One flash word, the unit of blank-verify and program. Eight bytes on every part.
pub const WORD_BYTES: u32 = 8;
/// One sector, the unit of erase.
pub const SECTOR_BYTES: u32 = 1024;

const FLASHCTL: u64 = 0x400C_D000;
const CMDEXEC: u64 = FLASHCTL + 0x1100;
const CMDTYPE: u64 = FLASHCTL + 0x1104;
const CMDCTL: u64 = FLASHCTL + 0x1108;
const CMDADDR: u64 = FLASHCTL + 0x1120;
const CMDBYTEN: u64 = FLASHCTL + 0x1124;
const CMDDATA0: u64 = FLASHCTL + 0x1130;
const CMDDATA1: u64 = FLASHCTL + 0x1134;
const CMDWEPROTA: u64 = FLASHCTL + 0x11D0;
const CMDWEPROTB: u64 = FLASHCTL + 0x11D4;
const STATCMD: u64 = FLASHCTL + 0x13D0;

/// `CMDCTL.ADDRXLATEOVR`, `ECCGENOVR` and `DATAVEREN`.
const ADDRXLATEOVR: u32 = 1 << 16;
const ECCGENOVR: u32 = 1 << 17;
const DATAVEREN: u32 = 1 << 21;

/// `CMDTYPE.COMMAND` in bits 0..3, `SIZE` in bits 4..7.
const PROGRAM: u32 = 0x1;
const ERASE: u32 = 0x2;
const CLEAR_STATUS: u32 = 0x5;
const BLANK_VERIFY: u32 = 0x6;
const ONE_WORD: u32 = 0x0 << 4;
const SECTOR: u32 = 0x4 << 4;

/// `CMDBYTEN`: eight data bytes, and the ECC byte in bit 8.
const ALL_BYTES: u32 = 0xFF;
const ECC_BYTE: u32 = 1 << 8;

const STAT_DONE: u32 = 1 << 0;
const STAT_PASS: u32 = 1 << 1;
const STAT_INPROGRESS: u32 = 1 << 2;
const STAT_FAILWEPROT: u32 = 1 << 4;
const STAT_FAILVERIFY: u32 = 1 << 5;
const STAT_FAILILLADDR: u32 = 1 << 6;
const STAT_FAILMODE: u32 = 1 << 7;
const STAT_FAILINVDATA: u32 = 1 << 8;

/// `CPUSS.CTL`: `PREFETCH`, `ICACHE` and `LITEN`, off across each command.
const CPUSS_CTL: u64 = 0x4040_1300;
const CPUSS_CACHES: u32 = 0x7;

const SYSCTL: u64 = 0x400A_F000;
/// Read after the cache disable, which does not take effect while a flash access is pending
/// (`CPU_ERR_02`). A transaction to another slave completes it. `CLKSTATUS` where the block has no
/// `SHUTDNSTORE`.
const SHUTDNSTORE0: u64 = SYSCTL + 0x1400;
const CLKSTATUS: u64 = SYSCTL + 0x1204;
/// `SECSTATUS.FLBANKSWP`: the two MAIN banks are running swapped.
const SECSTATUS: u64 = SYSCTL + 0x3048;
const FLBANKSWP: u32 = 1 << 12;

/// `FACTORYREGION.SRAMFLASH`: `MAINFLASH_SZ` in KiB in bits 0..12, `MAINNUMBANKS - 1` in 12..14.
const SRAMFLASH: u64 = 0x41C4_0018;

/// Sectors covered by one `CMDWEPROTB` bit.
const SECTORS_PER_WEPROTB_BIT: u32 = 8;

/// Long enough for a sector erase with a wide margin; the part takes milliseconds.
const DEADLINE: Duration = Duration::from_secs(2);

/// What a part's controller looks like, from the catalog.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Geometry {
    /// Bits in `CMDWEPROTA`, one sector each. Zero where the register is absent.
    pub weprota_bits: u8,
    /// Bits in `CMDWEPROTB`, eight sectors each.
    pub weprotb_bits: u8,
    pub has_ecc: bool,
    /// Whether SYSCTL has `SHUTDNSTORE`, which decides the register read after the cache disable.
    pub shutdnstore: bool,
    /// Whether the banks can run swapped, which moves the sector a `CMDWEPROTA` bit covers.
    pub bank_swap: bool,
}

/// This part's geometry, or `None` if the catalog does not know it.
///
/// `chip` is matched lowercased, as [`crate::Bench::chip`] returns it.
#[must_use]
pub fn geometry(chip: &str) -> Option<Geometry> {
    crate::mspm0_parts::FLASH
        .iter()
        .find(|(name, _)| *name == chip)
        .map(|(_, geometry)| *geometry)
}

/// A controller command.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Command {
    BlankVerify,
    Program,
    Erase,
}

impl std::fmt::Display for Command {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::BlankVerify => "blank-verify",
            Self::Program => "program",
            Self::Erase => "erase",
        })
    }
}

/// Why the controller refused a command, from `STATCMD`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Fault {
    /// The sector is write-protected by something other than this module: static protection from
    /// the boot configuration, usually.
    Protected,
    /// The word or sector did not reach the state asked for.
    Verify,
    IllegalAddress,
    /// A bank was left in a mode other than read.
    Mode,
    /// A program needed a stored zero to return to one, which only an erase can do.
    NotErased,
    /// No failure bit set, but no pass either. The whole `STATCMD`.
    Other(u32),
}

impl std::fmt::Display for Fault {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Protected => f.write_str("the sector is write-protected"),
            Self::Verify => f.write_str("it did not verify"),
            Self::IllegalAddress => f.write_str("the address is illegal"),
            Self::Mode => f.write_str("a bank is not in read mode"),
            Self::NotErased => f.write_str("the word is not erased"),
            Self::Other(status) => write!(f, "STATCMD {status:#05x}"),
        }
    }
}

fn fault(status: u32) -> Option<Fault> {
    if status & STAT_PASS != 0 {
        return None;
    }
    Some(if status & STAT_FAILWEPROT != 0 {
        Fault::Protected
    } else if status & STAT_FAILILLADDR != 0 {
        Fault::IllegalAddress
    } else if status & STAT_FAILMODE != 0 {
        Fault::Mode
    } else if status & STAT_FAILINVDATA != 0 {
        Fault::NotErased
    } else if status & STAT_FAILVERIFY != 0 {
        Fault::Verify
    } else {
        Fault::Other(status)
    })
}

/// The register access the controller needs, so the sequence can be tested without a part.
trait Bus {
    fn read(&mut self, address: u64) -> Result<u32, Error>;
    fn write(&mut self, address: u64, value: u32) -> Result<(), Error>;
}

impl Bus for Held<'_> {
    fn read(&mut self, address: u64) -> Result<u32, Error> {
        self.read_u32(address)
    }

    fn write(&mut self, address: u64, value: u32) -> Result<(), Error> {
        self.write_u32(address, value)
    }
}

/// Which protection register and bit covers a sector.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Protection {
    A(u32),
    B(u32),
}

/// The HAL's `unprotect`, as arithmetic.
///
/// `sectors` and `banks` are what the part reports in `FACTORYREGION`; `swapped` is `FLBANKSWP`.
fn protection(geometry: &Geometry, sector: u32, sectors: u32, banks: u32, swapped: bool) -> Option<Protection> {
    let per_bank = sectors / banks;
    let in_bank = sector % per_bank;
    let weprota = u32::from(geometry.weprota_bits);

    let protection = if weprota == 0 {
        Protection::B(in_bank / SECTORS_PER_WEPROTB_BIT)
    } else {
        // `CMDWEPROTA` covers physical bank 0, so a swap moves which sector an address reaches.
        let physical = if swapped {
            let half = sectors / 2;
            if sector >= half { sector - half } else { sector + half }
        } else {
            sector
        };
        if physical < weprota {
            Protection::A(physical)
        } else if banks == 1 {
            Protection::B((in_bank - weprota) / SECTORS_PER_WEPROTB_BIT)
        } else {
            Protection::B(in_bank / SECTORS_PER_WEPROTB_BIT)
        }
    };

    // A bit past the register's width protects nothing, and the command would fail as protected
    // with no hint why. The HAL asserts this in a debug build; here it is refused.
    match protection {
        Protection::B(bit) if bit >= u32::from(geometry.weprotb_bits) => None,
        other => Some(other),
    }
}

/// The command sequence over some [`Bus`].
struct Controller<B> {
    bus: B,
    geometry: Geometry,
    /// MAIN's sector count and bank count, as the part reports them.
    sectors: u32,
    banks: u32,
    swapped: bool,
    deadline: Duration,
    /// Set when a command did not finish. The controller may still own the bank, and the caches
    /// are still off, so the core must not run.
    wedged: bool,
}

/// MAIN's sector count, its bank count, and whether the banks are swapped, as the part reports them.
fn layout(bus: &mut impl Bus, geometry: &Geometry) -> Result<(u32, u32, bool), Error> {
    let sramflash = bus.read(SRAMFLASH)?;
    let sectors = (sramflash & 0x0FFF) * 1024 / SECTOR_BYTES;
    let banks = ((sramflash >> 12) & 0x3) + 1;
    let swapped = geometry.bank_swap && bus.read(SECSTATUS)? & FLBANKSWP != 0;
    Ok((sectors, banks, swapped))
}

impl<B: Bus> Controller<B> {
    #[cfg(test)]
    fn new(mut bus: B, geometry: Geometry) -> Result<Self, Error> {
        let (sectors, banks, swapped) = layout(&mut bus, &geometry)?;
        Ok(Self {
            bus,
            geometry,
            sectors,
            banks,
            swapped,
            deadline: DEADLINE,
            wedged: false,
        })
    }

    fn size(&self) -> u32 {
        self.sectors * SECTOR_BYTES
    }

    fn check(&self, address: u32, len: u32, alignment: u32) -> Result<(), Error> {
        let reason = if !address.is_multiple_of(alignment) {
            "is not aligned to the command's unit"
        } else if u64::from(address) + u64::from(len) > u64::from(self.size()) {
            "is outside MAIN"
        } else {
            return Ok(());
        };
        Err(Error::FlashAddress { address, reason })
    }

    fn is_blank(&mut self, address: u32) -> Result<bool, Error> {
        self.check(address, WORD_BYTES, WORD_BYTES)?;
        match self.command(Command::BlankVerify, address, None) {
            Ok(()) => Ok(true),
            Err(Error::FlashCommand {
                fault: Fault::Verify, ..
            }) => Ok(false),
            Err(error) => Err(error),
        }
    }

    fn program(&mut self, address: u32, word: [u8; 8]) -> Result<(), Error> {
        self.check(address, WORD_BYTES, WORD_BYTES)?;
        let low = u32::from_le_bytes([word[0], word[1], word[2], word[3]]);
        let high = u32::from_le_bytes([word[4], word[5], word[6], word[7]]);
        self.command(Command::Program, address, Some([low, high]))
    }

    fn erase(&mut self, address: u32) -> Result<(), Error> {
        self.check(address, SECTOR_BYTES, SECTOR_BYTES)?;
        self.command(Command::Erase, address, None)
    }

    /// One command, in the HAL's order. Clearing the status re-protects every sector, so the
    /// unprotect has to follow it, and both precede the execute.
    fn command(&mut self, command: Command, address: u32, data: Option<[u32; 2]>) -> Result<(), Error> {
        if self.wedged {
            return Err(Error::FlashWedged);
        }
        let _span = tracing::debug_span!("flashctl", %command, address).entered();

        let protection = protection(
            &self.geometry,
            address / SECTOR_BYTES,
            self.sectors,
            self.banks,
            self.swapped,
        )
        .ok_or(Error::FlashAddress {
            address,
            reason: "has no protection bit this part implements",
        })?;

        let cmdctl = self.bus.read(CMDCTL)?;
        self.bus
            .write(CMDCTL, (cmdctl & !(ADDRXLATEOVR | ECCGENOVR)) | DATAVEREN)?;

        let saved = self.bus.read(CPUSS_CTL)?;
        self.bus.write(CPUSS_CTL, saved & !CPUSS_CACHES)?;
        self.bus.read(if self.geometry.shutdnstore {
            SHUTDNSTORE0
        } else {
            CLKSTATUS
        })?;

        self.bus.write(CMDTYPE, CLEAR_STATUS)?;
        self.bus.write(CMDEXEC, 1)?;
        self.wait(command, address, |status| status & STAT_INPROGRESS == 0)?;

        self.bus.write(
            CMDTYPE,
            match command {
                Command::BlankVerify => BLANK_VERIFY | ONE_WORD,
                Command::Program => PROGRAM | ONE_WORD,
                Command::Erase => ERASE | SECTOR,
            },
        )?;
        if let Some([low, high]) = data {
            let ecc = if self.geometry.has_ecc { ECC_BYTE } else { 0 };
            self.bus.write(CMDBYTEN, ALL_BYTES | ecc)?;
            self.bus.write(CMDDATA0, low)?;
            self.bus.write(CMDDATA1, high)?;
        }
        self.bus.write(CMDADDR, address)?;
        let (register, bit) = match protection {
            Protection::A(bit) => (CMDWEPROTA, bit),
            Protection::B(bit) => (CMDWEPROTB, bit),
        };
        let protected = self.bus.read(register)?;
        self.bus.write(register, protected & !(1 << bit))?;

        self.bus.write(CMDEXEC, 1)?;
        // For `CMDDONE`, not for `CMDINPROGRESS` to clear: the latter takes a few cycles to assert.
        let status = self.wait(command, address, |status| status & STAT_DONE != 0)?;

        self.bus.write(CPUSS_CTL, saved)?;

        match fault(status) {
            None => Ok(()),
            Some(fault) => Err(Error::FlashCommand {
                command,
                address,
                fault,
            }),
        }
    }

    fn wait(&mut self, command: Command, address: u32, until: impl Fn(u32) -> bool) -> Result<u32, Error> {
        let started = Instant::now();
        loop {
            let status = self.bus.read(STATCMD)?;
            if until(status) {
                return Ok(status);
            }
            if started.elapsed() > self.deadline {
                self.wedged = true;
                return Err(Error::FlashTimeout { command, address });
            }
        }
    }
}

/// The flash controller, with the core halted for as long as this exists.
pub struct Flash<'a> {
    controller: Controller<Held<'a>>,
    /// Whether the core was running when this halted it, so dropping this should let it run.
    resume: bool,
}

impl<'a> Flash<'a> {
    /// Halt the core and take the controller.
    ///
    /// Fails on a part the catalog has no flash geometry for.
    pub fn new(mut held: Held<'a>) -> Result<Self, Error> {
        let geometry = geometry(held.chip()).ok_or_else(|| Error::NoFlashMap {
            chip: held.chip().to_owned(),
        })?;
        let resume = !matches!(held.status()?, CoreStatus::Halted(_));
        if resume {
            held.core().halt(Duration::from_millis(100))?;
        }
        // Past the halt, a failure has to go through the drop so the core runs again.
        let mut flash = Self {
            controller: Controller {
                bus: held,
                geometry,
                sectors: 0,
                banks: 1,
                swapped: false,
                deadline: DEADLINE,
                wedged: false,
            },
            resume,
        };
        let (sectors, banks, swapped) = layout(&mut flash.controller.bus, &geometry)?;
        flash.controller.sectors = sectors;
        flash.controller.banks = banks;
        flash.controller.swapped = swapped;
        Ok(flash)
    }

    /// MAIN's size in bytes, as the part reports it.
    #[must_use]
    pub fn size(&self) -> u32 {
        self.controller.size()
    }

    /// Whether the flash word at `address` is erased, or programmed all-ones.
    pub fn is_blank(&mut self, address: u32) -> Result<bool, Error> {
        self.controller.is_blank(address)
    }

    /// Program one flash word. `word[0]` lands at `address`.
    pub fn program(&mut self, address: u32, word: [u8; 8]) -> Result<(), Error> {
        self.controller.program(address, word)
    }

    /// Erase the sector at `address`, which must be a sector base.
    pub fn erase(&mut self, address: u32) -> Result<(), Error> {
        self.controller.erase(address)
    }

    /// Read flash, or anything else, while the core is halted.
    pub fn read(&mut self, address: u32, out: &mut [u8]) -> Result<(), Error> {
        self.controller.bus.read_bytes(u64::from(address), out)
    }

    /// Give the core back, running if it was running, and report whether that worked.
    ///
    /// Dropping does the same and swallows the error.
    pub fn release(mut self) -> Result<(), Error> {
        self.let_run()
    }

    fn let_run(&mut self) -> Result<(), Error> {
        if std::mem::take(&mut self.resume) {
            if self.controller.wedged {
                return Err(Error::FlashWedged);
            }
            self.controller.bus.resume()?;
        }
        Ok(())
    }
}

impl Drop for Flash<'_> {
    fn drop(&mut self) {
        if let Err(error) = self.let_run() {
            tracing::warn!(%error, "leaving the core halted");
        }
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;
    use std::time::Duration;

    use super::*;

    const L1306: Geometry = Geometry {
        weprota_bits: 32,
        weprotb_bits: 4,
        has_ecc: false,
        shutdnstore: true,
        bank_swap: false,
    };

    #[test]
    fn the_table_matches_the_catalog_for_the_parts_it_was_checked_on() {
        assert_eq!(geometry("mspm0l1306"), Some(L1306));
        assert_eq!(
            geometry("mspm0g3519"),
            Some(Geometry {
                weprota_bits: 0,
                weprotb_bits: 32,
                has_ecc: true,
                shutdnstore: true,
                bank_swap: true,
            })
        );
        assert_eq!(geometry("stm32f103"), None);
    }

    #[test]
    fn a_sector_under_weprota_is_its_own_bit() {
        assert_eq!(protection(&L1306, 0, 64, 1, false), Some(Protection::A(0)));
        assert_eq!(protection(&L1306, 31, 64, 1, false), Some(Protection::A(31)));
    }

    /// On one bank, `CMDWEPROTB` starts above the sectors `CMDWEPROTA` covers.
    #[test]
    fn past_weprota_one_bank_counts_from_its_end() {
        assert_eq!(protection(&L1306, 32, 64, 1, false), Some(Protection::B(0)));
        assert_eq!(protection(&L1306, 40, 64, 1, false), Some(Protection::B(1)));
        assert_eq!(protection(&L1306, 63, 64, 1, false), Some(Protection::B(3)));
    }

    /// On more than one bank it starts at each bank's base.
    #[test]
    fn past_weprota_two_banks_count_from_the_bank() {
        let l2228 = geometry("mspm0l2228").unwrap();
        assert_eq!(protection(&l2228, 128 + 40, 256, 2, false), Some(Protection::B(5)));
    }

    #[test]
    fn without_weprota_every_sector_is_in_weprotb() {
        let g3519 = geometry("mspm0g3519").unwrap();
        assert_eq!(protection(&g3519, 7, 512, 2, false), Some(Protection::B(0)));
        assert_eq!(protection(&g3519, 256 + 44, 512, 2, false), Some(Protection::B(5)));
    }

    #[test]
    fn a_swap_moves_the_weprota_bit_to_the_other_half() {
        let l2228 = geometry("mspm0l2228").unwrap();
        assert_eq!(protection(&l2228, 128 + 3, 256, 2, true), Some(Protection::A(3)));
        assert_eq!(protection(&l2228, 3, 256, 2, true), Some(Protection::B(0)));
    }

    #[test]
    fn a_bit_past_weprotb_is_refused() {
        let narrow = Geometry {
            weprotb_bits: 2,
            ..L1306
        };
        assert_eq!(protection(&narrow, 48, 64, 1, false), None);
    }

    /// A controller as far as the sequence needs one: registers, a flash array, dynamic protection
    /// that re-arms on clear-status, and the refusals `DATAVEREN` gives.
    struct Fake {
        regs: HashMap<u64, u32>,
        flash: Vec<Option<[u32; 2]>>,
        /// Every register write, in order.
        log: Vec<(u64, u32)>,
        /// Commands that never finish.
        hang: bool,
    }

    impl Fake {
        fn new() -> Self {
            let mut regs = HashMap::new();
            regs.insert(SRAMFLASH, 64);
            regs.insert(CPUSS_CTL, 0x7);
            regs.insert(CMDWEPROTA, u32::MAX);
            regs.insert(CMDWEPROTB, u32::MAX);
            Self {
                regs,
                flash: vec![None; 64 * 1024 / 8],
                log: Vec::new(),
                hang: false,
            }
        }

        fn reg(&self, address: u64) -> u32 {
            self.regs.get(&address).copied().unwrap_or(0)
        }

        fn execute(&mut self) -> u32 {
            let kind = self.reg(CMDTYPE);
            if kind == CLEAR_STATUS {
                self.regs.insert(CMDWEPROTA, u32::MAX);
                self.regs.insert(CMDWEPROTB, u32::MAX);
                return 0;
            }
            if self.hang {
                return STAT_INPROGRESS;
            }
            assert_eq!(self.reg(CPUSS_CTL) & CPUSS_CACHES, 0, "caches on during a command");
            assert_ne!(self.reg(CMDCTL) & DATAVEREN, 0, "DATAVEREN not set");
            let address = self.reg(CMDADDR);
            let sector = address / SECTOR_BYTES;
            let unprotected = match protection(&L1306, sector, 64, 1, false).unwrap() {
                Protection::A(bit) => self.reg(CMDWEPROTA) & (1 << bit) == 0,
                Protection::B(bit) => self.reg(CMDWEPROTB) & (1 << bit) == 0,
            };
            if !unprotected {
                return STAT_DONE | STAT_FAILWEPROT;
            }
            let word = (address / WORD_BYTES) as usize;
            match kind {
                k if k == BLANK_VERIFY | ONE_WORD => match self.flash[word] {
                    None | Some([u32::MAX, u32::MAX]) => STAT_DONE | STAT_PASS,
                    Some(_) => STAT_DONE | STAT_FAILVERIFY,
                },
                k if k == PROGRAM | ONE_WORD => {
                    let new = [self.reg(CMDDATA0), self.reg(CMDDATA1)];
                    let old = self.flash[word].unwrap_or([u32::MAX, u32::MAX]);
                    if new[0] & !old[0] != 0 || new[1] & !old[1] != 0 {
                        return STAT_DONE | STAT_FAILINVDATA;
                    }
                    self.flash[word] = Some(new);
                    STAT_DONE | STAT_PASS
                }
                k if k == ERASE | SECTOR => {
                    let first = (sector * SECTOR_BYTES / WORD_BYTES) as usize;
                    self.flash[first..first + (SECTOR_BYTES / WORD_BYTES) as usize].fill(None);
                    STAT_DONE | STAT_PASS
                }
                other => panic!("unexpected CMDTYPE {other:#x}"),
            }
        }
    }

    impl Bus for Fake {
        fn read(&mut self, address: u64) -> Result<u32, Error> {
            Ok(self.reg(address))
        }

        fn write(&mut self, address: u64, value: u32) -> Result<(), Error> {
            self.log.push((address, value));
            self.regs.insert(address, value);
            if address == CMDEXEC {
                let status = self.execute();
                self.regs.insert(STATCMD, status);
            }
            Ok(())
        }
    }

    fn controller() -> Controller<Fake> {
        Controller::new(Fake::new(), L1306).unwrap()
    }

    #[test]
    fn the_part_reports_its_own_size() {
        let c = controller();
        assert_eq!((c.sectors, c.banks, c.size()), (64, 1, 64 * 1024));
    }

    #[test]
    fn program_then_blank_then_erase() {
        let mut c = controller();
        assert!(c.is_blank(0x2008).unwrap());
        c.program(0x2008, [1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        assert_eq!(c.bus.flash[0x2008 / 8], Some([0x0403_0201, 0x0807_0605]));
        assert!(!c.is_blank(0x2008).unwrap());
        assert!(c.is_blank(0x2010).unwrap(), "the neighbour is untouched");

        c.erase(0x2000).unwrap();
        assert!(c.is_blank(0x2008).unwrap());
    }

    #[test]
    fn a_program_that_needs_an_erase_is_refused() {
        let mut c = controller();
        c.program(0x2008, [0; 8]).unwrap();
        assert!(matches!(
            c.program(0x2008, [0xFF; 8]),
            Err(Error::FlashCommand {
                fault: Fault::NotErased,
                ..
            })
        ));
    }

    /// Clear-status re-protects everything, so the unprotect has to come after it; and the caches
    /// come back on after the command, at the value they had.
    #[test]
    fn the_order_is_the_controllers() {
        let mut c = controller();
        c.bus.regs.insert(CPUSS_CTL, 0x5);
        c.erase(0x2000).unwrap();
        let log = &c.bus.log;
        let at = |address: u64, value: Option<u32>| {
            log.iter()
                .position(|(a, v)| *a == address && value.is_none_or(|x| x == *v))
                .unwrap()
        };
        let clear = at(CMDTYPE, Some(CLEAR_STATUS));
        let unprotect = at(CMDWEPROTA, Some(!(1 << 8)));
        let execute = log.iter().rposition(|(a, _)| *a == CMDEXEC).unwrap();
        assert!(clear < unprotect && unprotect < execute);
        assert_eq!(log.last(), Some(&(CPUSS_CTL, 0x5)));
    }

    #[test]
    fn nonmain_and_misalignment_are_refused_before_the_controller_is_touched() {
        let mut c = controller();
        for (what, result) in [
            ("beyond MAIN", c.erase(64 * 1024)),
            ("misaligned sector", c.erase(0x2008)),
            ("misaligned word", c.program(0x2004, [0; 8])),
        ] {
            assert!(matches!(result, Err(Error::FlashAddress { .. })), "{what}");
        }
        assert!(c.bus.log.is_empty());
    }

    #[test]
    fn a_command_that_does_not_finish_wedges_the_controller() {
        let mut c = controller();
        c.deadline = Duration::from_millis(5);
        c.bus.hang = true;
        assert!(matches!(c.erase(0x2000), Err(Error::FlashTimeout { .. })));
        assert_eq!(c.bus.reg(CPUSS_CTL) & CPUSS_CACHES, 0, "the caches stay off");
        c.bus.hang = false;
        assert!(matches!(c.erase(0x2000), Err(Error::FlashWedged)));
    }
}
