//! Driving MSPM0 pins from the debugger, on any image and on any port.
//!
//! Ports are uniform — the block is at `0x400A_0000` plus `0x2000` each, on all 43 parts in the
//! catalog. The pin mux is not: which `PINCM` register a pin uses is a per-part table, so
//! everything here needs the chip name the session was attached with. See [`crate::mspm0_parts`].
//!
//! Device knowledge, not application knowledge: which register holds what, and which pins must
//! never be driven. Every address and field position below is from the metapac the HAL itself uses,
//! and the sequence is the one the HAL's own `set_as_output` performs.
//!
//! # It needs nothing from the firmware, and that took a correction
//!
//! The pin is muxed, enabled and driven by writing the peripheral's registers over SWD. So it works
//! on a production image, on a bring-up image, and on an image built before anyone wanted this.
//!
//! **It did not work with no image at all, and said nothing about it.** The bank's `PWREN` gates
//! every register below it, and an application's `init` is what sets it — so on a part halted at the
//! reset vector, or a blank one, every write here went to an isolated peripheral and was dropped
//! while the call returned `Ok`. Measured: `PWREN` reads `1` with the image running and `0` at the
//! reset vector, and the same drive takes in the first case and vanishes in the second.
//!
//! [`power_on`] closes it, and every entry point calls it. **Enabling an unpowered bank cannot
//! disturb an owner, because an unpowered bank has no owner** — which is what makes doing it
//! unconditionally safe rather than a thing to ask about. The bank's reset is *not* asserted: that
//! would clear the pin state of an application that is using it, and `PWREN` alone is enough.
//!
//! With that, driving a pin on an **erased** part works — confirmed on hardware. The core has to
//! stay halted for it: a blank part faults as soon as it is released and takes the access port with
//! it, so `reset_and_halt` and no resume is the sequence. The GPIO peripheral is a bus slave the
//! debugger reaches without the CPU's help, which is why the CPU having nowhere to go does not
//! matter.
//!
//! # And the firmware does not know it happened
//!
//! **This is the hazard, and it cannot be designed away.** A pin the firmware also uses has two
//! owners: its driver rewrites `DOUT`, `DOE` or the mux whenever it next touches that pin, and
//! which of the two wins is a race. Driving a pin the firmware owns is for seeing what happens,
//! not for holding a level. **Which pins a firmware owns is the caller's to know** — this crate has
//! no way to find out, and does not try.
//!
//! # One refusal, and only one
//!
//! The debug pins. Everything else goes through, including pins that will fight an external driver,
//! because a bench exists to try things and the tool has no way to know what is wired to a board.
//! The debug pins are different in kind: driving them removes the ability to undo it.

use crate::{Error, Held, Target};

/// `GPIOA`, from the metapac. Every port is this plus [`PORT_STRIDE`] per port.
const GPIO_BASE: u64 = 0x400A_0000;

/// Between one GPIO port and the next.
///
/// **Checked across all 43 parts in the catalog**: `GPIOA` at `0x400A_0000`, `GPIOB` at
/// `0x400A_2000` on the 29 that have one, `GPIOC` at `0x400A_4000` on the 8 that do. The register
/// offsets below the base are the block's rather than the device's, so a port is a base address and
/// nothing else.
const PORT_STRIDE: u64 = 0x2000;
/// `IOMUX`, from the metapac. `PINCM[n]` is at `+0x04 + n*4`.
const IOMUX: u64 = 0x4042_8000;

const DOUT31_0: u64 = 0x1280;
const DOUTSET31_0: u64 = 0x1290;
const DOUTCLR31_0: u64 = 0x12A0;
const DOE31_0: u64 = 0x12C0;
const DOESET31_0: u64 = 0x12D0;
const DOECLR31_0: u64 = 0x12E0;
const DIN31_0: u64 = 0x1380;

/// `GPIOA.GPRCM.PWREN`, from the metapac: the block is at `+0x800` and `PWREN` at `+0x00`.
const PWREN: u64 = 0x0800;
/// `PWREN.ENABLE`, bit 0.
const PWREN_ENABLE: u32 = 1;
/// `PWREN.KEY`, `0x26` in bits 31:24. A write without it is ignored.
const PWREN_KEY: u32 = 0x26 << 24;

/// `PINCM.PF` selecting the GPIO function. 1 on every MSPM0.
const GPIO_PF: u32 = 1;
const PF_MASK: u32 = 0x3F;
/// `PINCM.PC`, the connection enable.
const PC: u32 = 1 << 7;
/// `PINCM.PIPD` and `PIPU`, the internal pulls.
const PIPD: u32 = 1 << 16;
const PIPU: u32 = 1 << 17;
/// `PINCM.INENA`, the input buffer. Needed to read a pin, including one being driven.
const INENA: u32 = 1 << 18;
/// `PINCM.HIZ1`, open-drain.
const HIZ1: u32 = 1 << 25;

/// **`PA19` is `SWDIO` and `PA20` is `SWCLK`.**
///
/// Driving either ends the session that is driving it, and no further command can undo it because
/// the commands travel over those pins. A power cycle recovers the board; nothing recovers the
/// session. This is the one thing here that is refused rather than warned about.
pub const DEBUG_PINS: [u8; 2] = [19, 20];

/// One pin, as `port * 32 + index` — so `Pin(11)` is `PA11` and `Pin(32)` is `PB0`.
///
/// **The same flat encoding the HAL uses** for its own `PIN_PORT`, so a number means the same thing
/// on both sides of the wire. Port A is 0, which is what keeps every existing `Pin(n)` meaning what
/// it did.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub struct Pin(pub u8);

impl Pin {
    /// Build a pin from a port and an index within it.
    #[must_use]
    pub const fn on(port: u8, index: u8) -> Self {
        Self(port * 32 + index)
    }

    /// 0 for `PA`, 1 for `PB`, 2 for `PC`.
    #[must_use]
    pub const fn port(self) -> u8 {
        self.0 / 32
    }

    /// The pin's number within its port.
    #[must_use]
    pub const fn index(self) -> u8 {
        self.0 % 32
    }

    /// Base of the GPIO block this pin belongs to.
    const fn gpio(self) -> u64 {
        GPIO_BASE + self.port() as u64 * PORT_STRIDE
    }

    /// Address of this pin's `PINCM`, for the part this session is attached to.
    ///
    /// **A table, because it is not arithmetic.** `PINCM` equals the pin number plus one on 14 of
    /// the 43 MSPM0 parts and on the other 29 the difference ranges from -27 to +28. A second port
    /// continues the same numbering rather than restarting — on an `MSPM0L2228`, `PB0` is
    /// `PINCM12` — so neither a per-port base nor a constant offset recovers it. See
    /// [`crate::mspm0_parts`].
    ///
    /// **The one-based value is already the offset.** TI numbers these registers from one and the
    /// array starts one register into the block, so the address is `IOMUX + pincm * 4`. Adding a
    /// further `0x04` lands one register high on every pin — which reads as the neighbouring pin's
    /// mux, and looks entirely plausible because the neighbour is usually configured too.
    fn pincm(self, chip: &str) -> Result<u64, Error> {
        crate::mspm0_parts::pincm(chip, self.port(), self.index())
            .map(|pincm| IOMUX + pincm as u64 * 4)
            .ok_or_else(|| Error::NoPinMap {
                chip: chip.to_owned(),
                pin: self.to_string(),
            })
    }

    const fn mask(self) -> u32 {
        1 << self.index()
    }

    /// **`SWDIO` and `SWCLK` are port A**, so the port has to match as well as the number.
    #[must_use]
    pub const fn is_debug(self) -> bool {
        self.port() == 0 && (self.index() == DEBUG_PINS[0] || self.index() == DEBUG_PINS[1])
    }
}

impl std::fmt::Display for Pin {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "P{}{}", (b'A' + self.port()) as char, self.index())
    }
}

/// What a pin is doing, as the silicon has it.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct State {
    /// The whole `PINCM`, kept so a pin can be put back exactly as it was found.
    pub pincm: u32,
    /// `DOE`: the output driver is on.
    pub driving: bool,
    /// `DOUT`: what it would drive.
    pub output: bool,
    /// `DIN`: what the pad actually reads. `None` when the input buffer is off, which is the
    /// normal state of an output — and reporting `false` for it would be a wrong answer rather
    /// than a missing one.
    pub input: Option<bool>,
    /// `PINCM.PF`. 1 is GPIO; anything else means a peripheral owns the pin.
    pub function: u8,
    /// `PINCM.PC`. With this clear the pin is disconnected, which is how an analog net rests.
    pub connected: bool,
}

impl State {
    /// Which internal pull the pad has.
    ///
    /// **Both bits set is not a state the hardware offers**, and reading it as `Up` rather than
    /// refusing is deliberate: this reports what a register holds, and a pad configured by something
    /// else is exactly what it is for.
    #[must_use]
    pub const fn pull(&self) -> Pull {
        if self.pincm & PIPU != 0 {
            Pull::Up
        } else if self.pincm & PIPD != 0 {
            Pull::Down
        } else {
            Pull::None
        }
    }

    /// Whether the input buffer is on, which is what makes [`State::input`] a reading rather than
    /// `None`.
    #[must_use]
    pub const fn readable(&self) -> bool {
        self.pincm & INENA != 0
    }

    /// Whether a peripheral other than GPIO is muxed onto it.
    #[must_use]
    pub const fn peripheral_owns_it(&self) -> bool {
        self.connected && self.function as u32 != GPIO_PF
    }
}

/// Read one pin without disturbing it.
pub fn read(bench: &mut Held<'_>, pin: Pin) -> Result<State, Error> {
    power_on(bench, pin)?;
    let pincm = bench.read_u32(pin.pincm(bench.chip())?)?;
    let doe = bench.read_u32(pin.gpio() + DOE31_0)?;
    let dout = bench.read_u32(pin.gpio() + DOUT31_0)?;
    let din = bench.read_u32(pin.gpio() + DIN31_0)?;

    Ok(State {
        pincm,
        driving: doe & pin.mask() != 0,
        output: dout & pin.mask() != 0,
        input: (pincm & INENA != 0).then_some(din & pin.mask() != 0),
        function: (pincm & PF_MASK) as u8,
        connected: pincm & PC != 0,
    })
}

/// Power the GPIO bank if it is not already, and say whether that had to be done.
///
/// **Every entry point here calls this first**, because the failure it prevents is silent: an
/// unpowered bank swallows writes and reports nothing, so a drive appears to succeed and the pin
/// does not move.
///
/// Safe to call unconditionally. A bank that is powered is left alone, and one that is not has no
/// application using it — an application that had reached its pins would have powered it.
///
/// The bank's reset is deliberately not asserted. `PWREN` alone is enough, measured, and asserting
/// reset would clear the pin state of a firmware that owns pins in this bank.
pub fn power_on(bench: &mut Held<'_>, pin: Pin) -> Result<bool, Error> {
    power_port(bench, pin.port())
}

/// [`power_on`] for a whole port, which is what a sweep across one wants.
fn power_port(bench: &mut Held<'_>, port: u8) -> Result<bool, Error> {
    let base = GPIO_BASE + port as u64 * PORT_STRIDE;
    if bench.read_u32(base + PWREN)? & PWREN_ENABLE != 0 {
        return Ok(false);
    }
    bench.write_u32(base + PWREN, PWREN_KEY | PWREN_ENABLE)?;
    // The registers behind `PWREN` stay isolated for a few ULPCLK cycles and a write that lands in
    // that window is dropped — which is the same silent failure one layer down.
    std::thread::sleep(std::time::Duration::from_millis(1));
    Ok(true)
}

/// Read every pin the part brings out, in one pass per port.
///
/// Four register reads per port rather than four per pin, which is what makes a pin table
/// refreshable at a useful rate over SWD.
pub fn read_all(bench: &mut Held<'_>) -> Result<Vec<(Pin, State)>, Error> {
    let _span = tracing::debug_span!("gpio_read_all").entered();
    let chip = bench.chip().to_owned();
    let Some(pins) = crate::mspm0_parts::pins(&chip) else {
        return Err(Error::NoPinMap {
            chip,
            pin: "any".to_owned(),
        });
    };
    let pins: Vec<(u8, u8)> = pins.collect();

    // **One block transfer for the whole `PINCM` array, not one read per pin.** The array is
    // contiguous in `PINCM` *number*, which is what makes a single transfer possible even where the
    // pin-to-`PINCM` mapping is not ordered. On this link taking the core costs about three times
    // what moving four bytes does, so a loop of `read_u32` spent most of a sweep acquiring rather
    // than reading: measured at 288 ms as a loop against 46 ms this way, and 35 acquisitions
    // against 4.
    let highest = pins
        .iter()
        .filter_map(|(port, index)| crate::mspm0_parts::pincm(&chip, *port, *index))
        .max()
        .unwrap_or(0);
    let mut raw = vec![0u8; highest as usize * 4];
    if !raw.is_empty() {
        bench.read_bytes(IOMUX + 4, &mut raw)?;
    }

    let mut out = Vec::with_capacity(pins.len());
    let mut ports: Vec<u8> = pins.iter().map(|(port, _)| *port).collect();
    ports.dedup();
    for port in ports {
        power_port(bench, port)?;
        let base = GPIO_BASE + port as u64 * PORT_STRIDE;
        let doe = bench.read_u32(base + DOE31_0)?;
        let dout = bench.read_u32(base + DOUT31_0)?;
        let din = bench.read_u32(base + DIN31_0)?;

        for (_, index) in pins.iter().filter(|(p, _)| *p == port) {
            let pin = Pin::on(port, *index);
            // One-based, and the array starts one register in — so the byte offset is
            // `(pincm - 1) * 4` into a block that began at `IOMUX + 4`.
            let Some(number) = crate::mspm0_parts::pincm(&chip, port, *index) else {
                continue;
            };
            let at = (number as usize - 1) * 4;
            let pincm = u32::from_le_bytes([raw[at], raw[at + 1], raw[at + 2], raw[at + 3]]);
            out.push((
                pin,
                State {
                    pincm,
                    driving: doe & pin.mask() != 0,
                    output: dout & pin.mask() != 0,
                    input: (pincm & INENA != 0).then_some(din & pin.mask() != 0),
                    function: (pincm & PF_MASK) as u8,
                    connected: pincm & PC != 0,
                },
            ))
        }
    }
    out.sort_by_key(|(pin, _)| *pin);
    Ok(out)
}

/// Drive a pin, returning the state it was in so it can be put back.
///
/// The order is the HAL's own: set the level first, then mux the pin to GPIO, then enable the
/// output driver. **Enabling the driver last is what stops a glitch** — the pad is connected while
/// `DOUT` already holds the wanted level, so the first thing it drives is that level rather than
/// whatever `DOUT` happened to contain.
///
/// The input buffer is left on, so [`read`] keeps reporting what the pad is really at. That is
/// worth the microamps here: a driven pin reading back the opposite level is how contention with
/// something external announces itself, and with the buffer off there is nothing to see.
pub fn drive(bench: &mut Held<'_>, pin: Pin, level: bool) -> Result<State, Error> {
    if pin.is_debug() {
        return Err(Error::DebugPin { pin: pin.0 });
    }
    // `read` powers the bank, so this does not.
    let was = read(bench, pin)?;

    let set = if level { DOUTSET31_0 } else { DOUTCLR31_0 };
    bench.write_u32(pin.gpio() + set, pin.mask())?;

    let pincm = (was.pincm & !(PF_MASK | HIZ1)) | GPIO_PF | PC | INENA;
    bench.write_u32(pin.pincm(bench.chip())?, pincm)?;

    bench.write_u32(pin.gpio() + DOESET31_0, pin.mask())?;
    Ok(was)
}

/// Which internal pull a pad has, or should have.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Pull {
    /// Neither. What a driven net wants, and what a pad with something external on it wants.
    #[default]
    None,
    Up,
    Down,
}

impl Pull {
    pub const ALL: [Self; 3] = [Self::None, Self::Up, Self::Down];

    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::Up => "up",
            Self::Down => "down",
        }
    }

    /// Parse a name, for a command line.
    #[must_use]
    pub fn parse(text: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|pull| pull.name() == text)
    }

    /// The `PINCM` bits this sets.
    const fn bits(self) -> u32 {
        match self {
            Self::None => 0,
            Self::Up => PIPU,
            Self::Down => PIPD,
        }
    }
}

/// What `PINCM` becomes when a pad is made readable without being taken.
///
/// **A function rather than a line inside [`observe`]**, so a test can exercise the arithmetic
/// itself. Written inline, the only available test restates the same expression and passes for a
/// mask that is wrong in both places.
const fn pincm_observing(was: u32) -> u32 {
    was | PC | INENA
}

/// What `PINCM` becomes when a pad is taken as a GPIO input with `pull`.
const fn pincm_as_input(was: u32, pull: Pull) -> u32 {
    (was & !(PF_MASK | PIPU | PIPD | HIZ1)) | GPIO_PF | PC | INENA | pull.bits()
}

/// Make a pin readable **without taking it from whatever owns it**.
///
/// # The least invasive thing that answers "what is this pad at"
///
/// `INENA` is independent of the pin function, so the input buffer can be turned on for a pad a
/// peripheral is driving or capturing on, and [`read`] then reports the level while that peripheral
/// keeps working. That is the difference between this and [`input`]: this observes, and that takes.
///
/// **It is not free of side effects, and one of them matters.** A pad rests *disconnected* — `PC`
/// clear — and a disconnected pad reads nothing, so this sets `PC` as well. On a digital net that
/// costs nothing. On an analog one it puts the digital input buffer across the node, which is a
/// perturbation of the thing being measured: [`State::connected`] on the returned prior state is
/// what says whether that happened, and [`restore`] is what undoes it.
///
/// Pulls, the output driver and the mux are all left exactly as found.
pub fn observe(bench: &mut Held<'_>, pin: Pin) -> Result<State, Error> {
    if pin.is_debug() {
        return Err(Error::DebugPin { pin: pin.0 });
    }
    // `read` powers the bank, so this does not.
    let was = read(bench, pin)?;
    bench.write_u32(pin.pincm(bench.chip())?, pincm_observing(was.pincm))?;
    Ok(was)
}

/// Take a pin as a plain GPIO input, with `pull`.
///
/// **This takes the pin**, where [`observe`] borrows it: the mux goes to GPIO and the output driver
/// goes off, so whatever peripheral had it loses it until [`restore`]. Use it to read a net nothing
/// on the part owns, or to see what an external driver is doing to one it does.
///
/// **Safe against contention in the one direction that matters.** The output driver is cleared
/// before the mux moves, so there is no instant at which this pad drives a level chosen by whatever
/// was in `DOUT`. [`drive`] has to do it the other way round and says so.
pub fn input(bench: &mut Held<'_>, pin: Pin, pull: Pull) -> Result<State, Error> {
    if pin.is_debug() {
        return Err(Error::DebugPin { pin: pin.0 });
    }
    // `read` powers the bank, so this does not.
    let was = read(bench, pin)?;

    // Off first: a pad that stops driving before it changes function never drives an unintended
    // level, where the other order would put `DOUT` on the pin for the width of one bus write.
    bench.write_u32(pin.gpio() + DOECLR31_0, pin.mask())?;

    bench.write_u32(pin.pincm(bench.chip())?, pincm_as_input(was.pincm, pull))?;
    Ok(was)
}

/// Put a pin back exactly as `was` found it.
///
/// The output driver goes off before the mux is restored, for the same reason it went on last.
///
/// **No caller in this workspace yet**, and it stays because [`observe`] and [`input`] are written
/// as borrows — each says in its own documentation that this is what gives the pin back. A borrow
/// with no return is a different API, and a narrower one.
pub fn restore(bench: &mut Held<'_>, pin: Pin, was: &State) -> Result<(), Error> {
    // **The guard the other mutators have and this one did not.** `State` is public with public
    // fields, so a caller can hand this a pin it never read — and this is the one entry point that
    // would then reconfigure `PA19` or `PA20` and take the debug port down mid-session.
    if pin.is_debug() {
        return Err(Error::DebugPin { pin: pin.0 });
    }
    if !was.driving {
        power_on(bench, pin)?;
        bench.write_u32(pin.gpio() + DOECLR31_0, pin.mask())?;
    }
    bench.write_u32(pin.pincm(bench.chip())?, was.pincm)?;
    if was.driving {
        let set = if was.output { DOUTSET31_0 } else { DOUTCLR31_0 };
        bench.write_u32(pin.gpio() + set, pin.mask())?;
        bench.write_u32(pin.gpio() + DOESET31_0, pin.mask())?;
    }
    Ok(())
}

/// Stop driving a pin and leave it disconnected, which is where an unused net rests.
pub fn release(bench: &mut Held<'_>, pin: Pin) -> Result<(), Error> {
    if pin.is_debug() {
        return Err(Error::DebugPin { pin: pin.0 });
    }
    power_on(bench, pin)?;
    bench.write_u32(pin.gpio() + DOECLR31_0, pin.mask())?;
    // `PC` clear is `PC_UNCONNECTED` in TI's own naming, and is where an analog net rests. Pulls
    // are cleared with it so nothing is left holding the node.
    let pincm = bench.read_u32(pin.pincm(bench.chip())?)? & !(PC | INENA | PIPU | PIPD);
    bench.write_u32(pin.pincm(bench.chip())?, pincm)
}

#[cfg(test)]
mod tests {
    /// **The distinction the two operations exist for.** `observe` leaves the mux and the driver
    /// alone; `input` takes both. A test on the register arithmetic rather than on a part, because
    /// the failure is a bit written into the wrong field and that is visible here.
    #[test]
    fn observing_keeps_the_function_and_taking_the_pin_does_not() {
        // A pad a peripheral owns, driving, with a pull up.
        let owned = (7 << 0) | PC | PIPU;

        let observed = pincm_observing(owned);
        assert_eq!(observed & PF_MASK, 7, "observe must not move the mux");
        assert!(observed & INENA != 0);
        assert!(observed & PIPU != 0, "observe must not change the pull");

        let taken = pincm_as_input(owned, Pull::Down);
        assert_eq!(taken & PF_MASK, GPIO_PF, "input takes the pin");
        assert!(taken & PIPD != 0 && taken & PIPU == 0, "the old pull must not survive");
        assert!(taken & INENA != 0);
    }

    /// A pull is one bit or neither, never both — a mask that failed to clear the other one would
    /// leave a pad pulled two ways.
    #[test]
    fn a_pull_sets_one_bit_and_reads_back_as_itself() {
        for pull in Pull::ALL {
            let state = State {
                pincm: pull.bits(),
                driving: false,
                output: false,
                input: None,
                function: 1,
                connected: false,
            };
            assert_eq!(state.pull(), pull, "{}", pull.name());
        }
        assert_eq!(Pull::None.bits(), 0);
        assert_ne!(Pull::Up.bits(), Pull::Down.bits());
    }

    #[test]
    fn a_pull_survives_its_own_name() {
        for pull in Pull::ALL {
            assert_eq!(Pull::parse(pull.name()), Some(pull));
        }
        assert_eq!(Pull::parse("floating"), None);
    }

    use super::*;

    /// **The addresses this produced before the table existed**, on the part it was written
    /// against. The lookup replaced `IOMUX + 0x04 + n * 4`, and on an L1306 it has to agree with it
    /// exactly or the change was not a generalisation but a regression.
    ///
    /// Written a register out either way, a read returns the neighbouring pin's mux — which looks
    /// entirely plausible, because the neighbour is usually configured too.
    #[test]
    fn the_regular_part_keeps_the_addresses_the_arithmetic_gave() {
        for n in [0u8, 11, 27] {
            assert_eq!(
                Pin(n).pincm("mspm0l1306").unwrap(),
                IOMUX + 0x04 + n as u64 * 4,
                "PA{n}"
            );
        }
        // The off-by-one this cost an afternoon to find.
        assert_ne!(Pin(11).pincm("mspm0l1306").unwrap(), IOMUX + 0x04 + 12 * 4);
    }

    /// **The reason the table exists.** On this part `PA2` is `PINCM7`, so the arithmetic that is
    /// right for an L1306 lands four registers away.
    #[test]
    fn an_irregular_part_disagrees_with_the_arithmetic() {
        let real = Pin(2).pincm("mspm0l2228").unwrap();
        assert_eq!(real, IOMUX + 7 * 4);
        assert_ne!(real, IOMUX + 0x04 + 2 * 4);
    }

    /// A second port continues the same `PINCM` numbering rather than restarting, and lands in the
    /// next GPIO block.
    #[test]
    fn a_second_port_moves_both_the_block_and_the_pincm() {
        let pb0 = Pin::on(1, 0);
        assert_eq!(pb0.to_string(), "PB0");
        assert_eq!(pb0.gpio(), GPIO_BASE + PORT_STRIDE);
        assert_eq!(pb0.pincm("mspm0l2228").unwrap(), IOMUX + 12 * 4);
        assert_eq!(Pin(0).gpio(), GPIO_BASE);
    }

    /// **Refused rather than guessed.** A fallback would mux the wrong pad on 29 of the 43 parts.
    #[test]
    fn an_unknown_part_is_an_error() {
        assert!(Pin(0).pincm("stm32f103").is_err());
        assert!(Pin::on(2, 0).pincm("mspm0l1306").is_err());
    }

    #[test]
    fn the_debug_pins_are_the_two_that_carry_this_conversation() {
        assert!(Pin(19).is_debug());
        assert!(Pin(20).is_debug());
        assert!(!Pin(18).is_debug());
        assert!(!Pin(21).is_debug());
    }

    /// A pin with a peripheral muxed onto it is not a free pin, and `PF == 1` is GPIO rather than
    /// "no peripheral" — so an unconnected pin must not read as owned.
    #[test]
    fn peripheral_ownership_needs_both_connected_and_a_non_gpio_function() {
        let owned = State {
            pincm: 0,
            driving: false,
            output: false,
            input: None,
            function: 5,
            connected: true,
        };
        assert!(owned.peripheral_owns_it());

        let gpio = State { function: 1, ..owned };
        assert!(!gpio.peripheral_owns_it());

        let disconnected = State {
            connected: false,
            ..owned
        };
        assert!(!disconnected.peripheral_owns_it());
    }
}
