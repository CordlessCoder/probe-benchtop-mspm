//! The MSPM0's application mailbox, from the host's end.
//!
//! One 32-bit word each way between a debug probe and the running CPU, over the SWD pair and no
//! other pins. It is the channel for asking a firmware to *do* something, where the rest of this
//! crate can only set a value it reads later.
//!
//! # It is an access port, not memory
//!
//! **A memory write to the DEBUGSS peripheral does not reach this.** The same two buffers are
//! memory-mapped from the CPU's side and reachable only through **SEC-AP, access port 2** from the
//! host's — one pair of buffers, two windows. So this drives raw AP registers rather than going
//! through the memory interface everything else here uses.
//!
//! SEC-AP is also where the boot ROM's recovery mailbox lives, which is a different conversation on
//! the same port: those commands are serviced out of a `BOOTRST` by code that is not the
//! application. Nothing here touches them.
//!
//! # The register names are the probe's
//!
//! `TXDATA` is what the probe *transmits*, so a host writes it and the CPU reads it. `RXDATA` is
//! what the probe receives. A driver that read `TX` as "out of the CPU" would have both directions
//! inverted, and the failure is a mailbox that looks dead rather than one that errors — so
//! [`Mailbox::send`] writes `TXDATA` and [`Mailbox::try_receive`] reads `RXDATA`, and each touches
//! the register whose name is the opposite of the method's.
//!
//! # Flow control, and there is no queue
//!
//! One word deep each way. `TXCTL.TRANSMIT` is set by a host write and clears **only** when the CPU
//! reads `TXDATA`; `RXCTL.RECEIVE` is set by a CPU write and clears only when the host reads
//! `RXDATA`. Neither side can clear the other's flag by any route but reading the data, and there
//! is no acknowledge register.
//!
//! That shape decides what this is good for. A word carries a request or a verdict well. Anything
//! wider is a handshake per word — a protocol, with framing and sequencing, which is the thing this
//! crate exists to not have. **Send a verdict here and read the detail from memory by symbol**: the
//! mailbox is for synchronisation, and memory is for data.
//!
//! # It works, measured 2026-08-31
//!
//! Before that day nothing had driven this mailbox from a host in either direction, and the CPU end
//! had never taken a word a probe put there. Now both have. On an MSPM0L1306, with a firmware that
//! polls `try_receive` every 500 ms:
//!
//! | | |
//! | --- | --- |
//! | SEC-AP `IDR` | `0x002E0000` |
//! | host writes `TXDATA` | `TXCTL.TRANSMIT` sets |
//! | ~400 ms later | `TRANSMIT` clears, so the CPU read it |
//! | the CPU replies | `RXCTL.RECEIVE` sets, and the exact word it sent comes back |
//!
//! The word that returned was the firmware's own computed reply rather than anything left in a
//! buffer, which is what makes it a channel and not an echo.
//!
//! **`RXIFG` fires when the probe reads, not when the target writes** — SLAU847 table 35-9 rather
//! than table 35-6, which contradict each other. Measured by clearing `CPU_INT.ICLR` first: the
//! flag was clear after the target's write and set after this host's read. Without that clear the
//! answer comes out backwards, because the bits latch and nothing had ever cleared them, so
//! everything reads set and "was it already set" answers a different question.

use probe_rs::architecture::arm::FullyQualifiedApAddress;

use crate::{Bench, Error};

/// SEC-AP, which carries both this mailbox and the boot ROM's.
const SEC_AP: u8 = 2;

/// What the probe transmits and the CPU reads.
const TXDATA: u64 = 0x00;
/// `TRANSMIT` in bit 0: a word is waiting for the CPU.
const TXCTL: u64 = 0x04;
/// What the CPU writes and the probe reads.
const RXDATA: u64 = 0x08;
/// `RECEIVE` in bit 0: a word is waiting for the host.
const RXCTL: u64 = 0x0C;
/// The access port's identification register, for proving the port is the one expected.
const IDR: u64 = 0x0FC;

/// Bit 0 of both control registers, set by the writer and cleared by the reader.
const PENDING: u32 = 1 << 0;

/// The mailbox on an attached target.
pub struct Mailbox<'a> {
    bench: &'a mut Bench,
    ap: FullyQualifiedApAddress,
}

impl<'a> Mailbox<'a> {
    /// Take the mailbox on SEC-AP.
    pub fn new(bench: &'a mut Bench) -> Self {
        Self {
            bench,
            ap: FullyQualifiedApAddress::v1_with_default_dp(SEC_AP),
        }
    }

    /// The access port's `IDR`.
    ///
    /// **Read this before believing anything else here.** Every other register on this port reads
    /// as some value whether or not the port is what it is thought to be, so a wrong `SEC_AP` gives
    /// plausible zeros rather than an error — the same silent shape as a stale ELF, one layer down.
    pub fn idr(&mut self) -> Result<u32, Error> {
        self.read(IDR)
    }

    /// Whether a word this host sent is still waiting for the CPU to collect it.
    pub fn send_pending(&mut self) -> Result<bool, Error> {
        Ok(self.read(TXCTL)? & PENDING != 0)
    }

    /// Whether the CPU has left a word for this host.
    pub fn receive_pending(&mut self) -> Result<bool, Error> {
        Ok(self.read(RXCTL)? & PENDING != 0)
    }

    /// Put a word in front of the CPU.
    ///
    /// Fails rather than overwriting when one is already pending, because there is no queue and the
    /// alternative is losing a message the target has not read yet.
    pub fn send(&mut self, word: u32) -> Result<(), Error> {
        if self.send_pending()? {
            return Err(Error::MailboxBusy);
        }
        self.write(TXDATA, word)
    }

    /// Take the CPU's word, or `None` if it has not left one.
    ///
    /// **Reading is what clears the flag**, so this consumes the word. There is no way to look
    /// without taking.
    pub fn try_receive(&mut self) -> Result<Option<u32>, Error> {
        if !self.receive_pending()? {
            return Ok(None);
        }
        self.read(RXDATA).map(Some)
    }

    fn read(&mut self, address: u64) -> Result<u32, Error> {
        let ap = self.ap.clone();
        let arm = self.bench.session().get_arm_interface()?;
        Ok(arm.read_raw_ap_register(&ap, address)?)
    }

    fn write(&mut self, address: u64, value: u32) -> Result<(), Error> {
        let ap = self.ap.clone();
        let arm = self.bench.session().get_arm_interface()?;
        Ok(arm.write_raw_ap_register(&ap, address, value)?)
    }
}
