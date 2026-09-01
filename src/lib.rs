//! Drive a Cortex-M board over SWD from a program rather than a shell.
//!
//! Attach, reset, prove the core is running, resolve a symbol out of the ELF, and read or write it
//! while the core runs. That is the whole of it, and it is enough to sweep a parameter without
//! recompiling: put the parameter in a `static` the firmware reads volatilely, and this writes it.
//!
//! # It knows nothing about any particular firmware
//!
//! It is handed an ELF path and a symbol name. Which symbols exist, what they mean, and what a
//! sweep of one is for all belong to the caller. That boundary is why this crate lives outside any
//! firmware repository, and it is what lets two unrelated projects use it.
//!
//! # The ELF is the schema
//!
//! There is no protocol here — no framing, no command identifiers, no version negotiation, and no
//! generated header anybody keeps in step by hand. The symbol table cannot drift from the image
//! because it *is* the image. See [`Symbols`].
//!
//! # No GUI dependency, ever
//!
//! A CI runner links this directly. Anything that needs a window belongs above it.

use std::path::{Path, PathBuf};
use std::time::Duration;

pub use probe_rs::CoreStatus;
use probe_rs::probe::DebugProbeSelector;
use probe_rs::probe::list::Lister;
use probe_rs::{MemoryInterface, Permissions, Session};

pub mod embassy_mspm0;
mod image;
pub mod mspm0_gpio;
pub mod mspm0_mailbox;
pub mod log;
mod symbols;
mod value;

pub use image::Verify;
pub use symbols::{Symbol, Symbols};
pub use value::Value;

/// Everything that can go wrong, named by what a user did rather than by what a layer returned.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("cannot read {path}")]
    ElfRead {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },

    #[error("{path} is not an ELF this can read")]
    ElfParse {
        path: PathBuf,
        #[source]
        source: object::Error,
    },

    #[error("no symbol named `{name}`{}", suggest(.near))]
    NoSuchSymbol { name: String, near: Vec<String> },

    /// The ELF says this symbol is a different width from the type asked for.
    ///
    /// Worth its own variant because the alternative is a silent partial write: poking a `u32` at
    /// a one-byte `static` clobbers three bytes of whatever the linker put next to it, and nothing
    /// downstream would report it.
    #[error("`{name}` is {actual} bytes in the ELF, and was read as a {wanted}-byte {type_name}")]
    WidthMismatch {
        name: String,
        actual: u64,
        wanted: u64,
        type_name: &'static str,
    },

    #[error("`{name}` is at {address:#010x}, which is not {alignment}-byte aligned")]
    Misaligned {
        name: String,
        address: u64,
        alignment: u64,
    },

    /// A write that did not stick.
    ///
    /// Reaching this means the address is real enough to write and read without an error, and the
    /// value still did not change — flash rather than RAM, or a region the core is holding.
    #[error("wrote {wrote:#x} to `{name}` and read back {read:#x}")]
    PokeDidNotStick { name: String, wrote: u64, read: u64 },

    /// probe-rs failed while attaching.
    ///
    /// Carries a hint the underlying message does not: on this family a locked-up core drops the
    /// power domain its access port lives in, so *every* way of locking a core up presents as a
    /// transport fault and nothing in the message says "core".
    #[error(
        "cannot attach{}\n\nOn MSPM0 a locked-up core takes its access port with it, so a fault \
         here may be the target rather than the probe: a faulting image, a blank main flash, or a \
         stack overflow. Power-cycling the board and attaching under reset is the way in.",
        maybe(.chip)
    )]
    Attach {
        chip: Option<String>,
        #[source]
        source: probe_rs::Error,
    },

    /// The ELF does not describe what is on the part.
    ///
    /// **The most important error in this crate**, because it is the one whose absence is silent.
    /// Every symbol still resolves, every address is still plausible, and every reading is of
    /// whatever the previous image left there.
    #[error(
        "{path} is not the image on this target: at {address:#010x} (segment offset {offset}) the \
         ELF says {expected:#04x} and the target holds {actual:#04x}. Flash this ELF, or point at \
         the one that was."
    )]
    ImageMismatch {
        path: PathBuf,
        address: u64,
        offset: usize,
        expected: u8,
        actual: u8,
    },

    #[error("{path} has no loadable segments")]
    NothingLoadable { path: PathBuf },

    #[error("{path} declares a defmt version this decoder does not speak")]
    DefmtTable {
        path: PathBuf,
        #[source]
        source: anyhow::Error,
    },

    /// No usable RTT control block.
    ///
    /// Usually the image has no RTT in it — a production build — rather than anything being wrong.
    #[error("no RTT: {detail}")]
    NoRtt { detail: String },

    #[error("this image has {len} RTT up-channel(s), so there is no channel {index}")]
    NoSuchChannel { index: usize, len: usize },

    /// The defmt stream is out of step with the ELF.
    ///
    /// Almost always a stale ELF rather than corruption, and the image check at attach is what
    /// normally catches that first.
    #[error("the defmt stream did not decode against this ELF")]
    MalformedDefmt,

    /// Driving a debug pin.
    ///
    /// The one thing refused outright: the command would travel over the pin it reconfigures, so
    /// nothing could undo it. Everything else a bench asks for goes through.
    #[error(
        "PA{pin} is SWDIO or SWCLK. Driving it would end this session over the pin it reconfigures, \
         and no further command could undo it — only a power cycle."
    )]
    DebugPin { pin: u8 },

    /// A word is already waiting for the CPU.
    ///
    /// The mailbox is one word deep with no queue, so sending over a pending word would lose it.
    #[error("a word is already waiting for the target to collect")]
    MailboxBusy,

    /// Programming the part.
    ///
    /// **The image on the part is not the ELF any more, whatever this says.** A failure here can
    /// land anywhere between "nothing was erased" and "half the sectors are written", so a caller
    /// that carries on reading symbols is reading an image that does not exist. Re-program, or
    /// re-attach; do not continue.
    #[error("cannot program {path}")]
    Flash {
        path: PathBuf,
        #[source]
        source: Box<probe_rs::flashing::FileDownloadError>,
    },

    /// The chip's flash could not be erased.
    ///
    /// **Names no path, because no file was involved.** An erase is the target's own algorithm run
    /// over the regions its description declares, and a failure here leaves the part somewhere
    /// between untouched and partly blank — the same "do not carry on reading symbols" hazard as
    /// [`Error::Flash`], from an operation that has no ELF to blame.
    #[error("cannot erase the target's flash")]
    Erase {
        #[source]
        source: Box<probe_rs::flashing::FlashError>,
    },

    #[error(transparent)]
    Arm(#[from] probe_rs::architecture::arm::ArmError),

    #[error(transparent)]
    Rtt(#[from] probe_rs::rtt::Error),

    #[error(transparent)]
    Probe(#[from] probe_rs::Error),

    #[error("no probe matching `{selector}`")]
    NoSuchProbe {
        selector: String,
        #[source]
        source: probe_rs::probe::DebugProbeError,
    },

    /// The selector string did not parse.
    ///
    /// The cause is a `String` rather than the underlying error because that error is not
    /// nameable: `DebugProbeSelector`'s `FromStr::Err` is `pub` inside a private module and never
    /// re-exported, so no caller outside probe-rs can write its type. Reported upstream.
    #[error("`{selector}` is not a probe selector: {cause}")]
    BadSelector { selector: String, cause: String },

    /// The probe itself, before there is a session to blame.
    #[error(transparent)]
    DebugProbe(#[from] probe_rs::probe::DebugProbeError),
}

fn suggest(near: &[String]) -> String {
    if near.is_empty() {
        String::new()
    } else {
        format!(". Did you mean: {}", near.join(", "))
    }
}

fn maybe(chip: &Option<String>) -> String {
    chip.as_deref().map(|c| format!(" to {c}")).unwrap_or_default()
}

/// How to reach a board.
///
/// A struct rather than four arguments, because a bench acquires these one at a time and a caller
/// that names them positionally gets the probe and the chip the wrong way round exactly once.
#[derive(Clone, PartialEq, Eq, Debug, Default)]
pub struct Attach {
    /// probe-rs chip name, e.g. `MSPM0L1306`.
    pub chip: String,
    /// probe-rs probe selector, e.g. `0451:bef3-5:ML130001`.
    ///
    /// **Name it whenever more than one probe is on the bus.** Attaching without a selector takes
    /// whichever probe is found first, and asking the wrong part what chip it is can be
    /// destructive on families where the answer involves a mass erase.
    pub probe: Option<String>,
    /// SWD rather than JTAG unless overridden. The default is deliberate: some probes default to
    /// JTAG and then fail in a way that reads as a wiring fault.
    pub protocol: Option<probe_rs::probe::WireProtocol>,
    pub speed_khz: Option<u32>,
    /// How much of the ELF to check against the part. Every loadable byte, by default.
    ///
    /// **On by default because the failure it catches is silent.** See [`Verify`].
    pub verify: Verify,
    /// Let the attach recover a part whose access port has stopped answering, by erasing it.
    ///
    /// **Off by default, and the default is the surprising one to live with.** A part with a blank
    /// or faulting MAIN takes its access port down, and the only way back in is the boot ROM's mass
    /// erase — which probe-rs will run during the attach, but only with this. Without it, attaching
    /// to a part you have just erased fails with a permissions error rather than recovering.
    ///
    /// It is still off by default because the recovery is silent and destroys whatever is on the
    /// part. Set it where the caller has already decided that: after an erase it asked for, or on a
    /// bring-up flow whose whole purpose is to get a blank part running.
    ///
    /// Reaches only the path taken when the access port is already dead. An ordinary attach to a
    /// working part never gets there, so this changes nothing for one.
    pub allow_erase_all: bool,
}

/// Which part of a flash a [`Progress`] is about.
///
/// **Not `probe_rs`'s own operation type.** That one carries no `PartialEq`, so a caller cannot ask
/// whether the phase has changed, and a probe-rs bump adding a variant would break every match on
/// it. Mapping once here is the whole of what a front end needs.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Phase {
    /// Reading back what an erased sector held, so bytes the image does not write survive it.
    Fill,
    Erase,
    Program,
    /// Reading back what was written and comparing it. Only when `verify` is set, which
    /// [`Bench::program`] sets.
    Verify,
    /// Writing straight to RAM, which is the flash algorithm being loaded rather than the image.
    Ram,
}

impl Phase {
    /// How many there are, for a table indexed by [`Phase::index`].
    const COUNT: usize = 5;

    /// A slot in such a table.
    #[must_use]
    pub fn index(self) -> usize {
        match self {
            Self::Fill => 0,
            Self::Erase => 1,
            Self::Program => 2,
            Self::Verify => 3,
            Self::Ram => 4,
        }
    }

    /// What to call it, in the lower case a status line wants.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::Fill => "filling",
            Self::Erase => "erasing",
            Self::Program => "programming",
            Self::Verify => "verifying",
            Self::Ram => "loading the algorithm",
        }
    }
}

impl From<probe_rs::flashing::ProgressOperation> for Phase {
    fn from(operation: probe_rs::flashing::ProgressOperation) -> Self {
        use probe_rs::flashing::ProgressOperation as O;
        match operation {
            O::Fill => Self::Fill,
            O::Erase => Self::Erase,
            O::Program => Self::Program,
            O::Verify => Self::Verify,
            O::Ram => Self::Ram,
        }
    }
}

impl std::fmt::Display for Phase {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.name())
    }
}

/// How far through one phase of a flash it is.
///
/// See [`Bench::program_watching`], and note that the phases are not one bar.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Progress {
    pub phase: Phase,
    /// Bytes this phase has covered.
    pub done: u64,
    /// What this phase covers in total, where the algorithm published one.
    ///
    /// `None` is a phase of indeterminate size, and a front end showing a bar for one is showing a
    /// number it made up.
    pub total: Option<u64>,
}

impl Progress {
    /// How far through this phase is, in `0.0..=1.0`.
    ///
    /// `None` where the total is unknown, and `1.0` where it is zero — a phase with nothing to do
    /// is finished rather than at the start of itself.
    #[must_use]
    pub fn fraction(&self) -> Option<f32> {
        let total = self.total?;
        if total == 0 {
            return Some(1.0);
        }
        #[expect(clippy::cast_precision_loss, reason = "a flash image is far short of a float's exact range")]
        Some((self.done.min(total) as f32) / (total as f32))
    }
}

/// One debug probe on the bus, as something a person can choose between.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct ProbeChoice {
    /// What to put in [`Attach::probe`], in the form probe-rs parses.
    pub selector: String,
    /// What the probe calls itself.
    pub name: String,
    /// Its serial, where it has one.
    ///
    /// **This is what distinguishes two probes of the same model**, and it is the field a selector
    /// needs to be unambiguous. A probe without one cannot be told apart from its twin.
    pub serial: Option<String>,
}

impl std::fmt::Display for ProbeChoice {
    /// Name, then serial, then the selector — most recognisable first.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name)?;
        if let Some(serial) = &self.serial {
            write!(f, " ({serial})")?;
        }
        write!(f, " — {}", self.selector)
    }
}

/// Every debug probe currently on the bus.
///
/// **Needs no session and cannot fail**, so a caller with no working attach can still offer a
/// choice — which is the case that matters, since the usual reason to want this list is that the
/// probe named at startup was the wrong one or was not plugged in.
///
/// The order is the lister's, which is the order an unnamed attach would pick from.
#[must_use]
pub fn probes() -> Vec<ProbeChoice> {
    Lister::new()
        .list_all()
        .into_iter()
        .map(|info| ProbeChoice {
            selector: probe_rs::probe::DebugProbeSelector::from(&info).to_string(),
            name: info.identifier.clone(),
            serial: info.serial_number.clone(),
        })
        .collect()
}

/// Turn one `probe_rs` progress event into a [`Progress`], accumulating as it goes.
///
/// **Shared because there are two callers**, programming and erasing, and the accumulation is the
/// same arithmetic — the totals arrive up front in `AddProgressBar` and the sizes add up after.
/// Written twice it is written differently eventually, and the difference shows as a bar that fills
/// at a different rate depending on which operation is running.
fn report(
    event: probe_rs::flashing::ProgressEvent,
    totals: &mut [Option<u64>; Phase::COUNT],
    done: &mut u64,
    watch: &mut impl FnMut(Progress),
) {
    use probe_rs::flashing::ProgressEvent as E;
    match event {
        E::AddProgressBar { operation, total } => totals[Phase::from(operation).index()] = total,
        E::Started(operation) => {
            *done = 0;
            let phase = Phase::from(operation);
            watch(Progress {
                phase,
                done: *done,
                total: totals[phase.index()],
            });
        }
        E::Progress { operation, size, .. } => {
            *done = done.saturating_add(size);
            let phase = Phase::from(operation);
            watch(Progress {
                phase,
                done: *done,
                total: totals[phase.index()],
            });
        }
        // **A phase that ends is reported at its total rather than at what was counted.** The
        // algorithm rounds a page to its own granularity, so the sum of the reports can fall short
        // of the total it published and leave a bar that never fills.
        E::Finished(operation) => {
            let phase = Phase::from(operation);
            let total = totals[phase.index()];
            watch(Progress {
                phase,
                done: total.unwrap_or(*done),
                total,
            });
        }
        E::FlashLayoutReady { .. } | E::Failed(_) | E::DiagnosticMessage { .. } => {}
    }
}

/// An attached board, with its ELF.
pub struct Bench {
    session: Session,
    symbols: Symbols,
    elf: PathBuf,
    resume_on_drop: bool,
}

impl Bench {
    /// Attach to a running target and read its ELF.
    ///
    /// **This does not reset.** A bench session usually wants to look at a board that is already
    /// doing something, and a reset is the more surprising default of the two.
    ///
    /// **It does check the ELF against the part**, and leaves the core as it found it otherwise.
    /// Note that attaching itself can halt a running core, depending on the probe and the debug
    /// sequence — [`Bench::status`] says whether it did and [`Bench::resume`] is the way back.
    pub fn attach(attach: &Attach, elf: &Path) -> Result<Self, Error> {
        let symbols = Symbols::load(elf)?;

        let lister = Lister::new();
        let probe = match &attach.probe {
            Some(selector) => {
                let selector: DebugProbeSelector =
                    selector.parse().map_err(|cause| Error::BadSelector {
                        selector: selector.clone(),
                        cause: format!("{cause}"),
                    })?;
                lister.open(selector).map_err(|source| Error::NoSuchProbe {
                    selector: attach.probe.clone().unwrap_or_default(),
                    source,
                })?
            }
            None => {
                let all = lister.list_all();
                let first = all.first().ok_or_else(|| Error::NoSuchProbe {
                    selector: "<any>".to_owned(),
                    source: probe_rs::probe::DebugProbeError::ProbeCouldNotBeCreated(
                        probe_rs::probe::ProbeCreationError::NotFound,
                    ),
                })?;
                lister
                    .open(first.clone())
                    .map_err(|source| Error::NoSuchProbe {
                        selector: "<any>".to_owned(),
                        source,
                    })?
            }
        };

        let mut probe = probe;
        probe.select_protocol(
            attach
                .protocol
                .unwrap_or(probe_rs::probe::WireProtocol::Swd),
        )?;
        if let Some(khz) = attach.speed_khz {
            probe.set_speed(khz)?;
        }

        let session = probe
            .attach(attach.chip.clone(), {
                let permissions = Permissions::new();
                if attach.allow_erase_all {
                    permissions.allow_erase_all()
                } else {
                    permissions
                }
            })
            .map_err(|source| Error::Attach {
                chip: Some(attach.chip.clone()),
                source,
            })?;

        let mut bench = Self {
            session,
            symbols,
            elf: elf.to_owned(),
            resume_on_drop: true,
        };

        {
            let mut core = bench.session.core(0)?;
            image::verify(&mut core, elf, attach.verify)?;
        }

        Ok(bench)
    }

    /// The ELF this was attached with.
    pub fn elf(&self) -> &Path {
        &self.elf
    }

    pub fn symbols(&self) -> &Symbols {
        &self.symbols
    }

    /// What probe-rs says the core is doing, without disturbing it.
    ///
    /// **Non-invasive, and that is the point.** The other way to prove a core is running is to halt
    /// it twice and watch the program counter move, which costs two halts — on firmware with a
    /// timing budget that is not an observation, it is an intervention. Use [`Bench::prove_running`]
    /// only when this is not enough.
    pub fn status(&mut self) -> Result<CoreStatus, Error> {
        let mut core = self.session.core(0)?;
        Ok(core.status()?)
    }

    /// Whether the core is executing or asleep, as opposed to halted, locked up or unknown.
    ///
    /// `Sleeping` counts. A part that has entered a low-power mode between interrupts is working
    /// exactly as intended, and treating it as dead is the mistake this exists to prevent.
    pub fn is_live(&mut self) -> Result<bool, Error> {
        Ok(matches!(
            self.status()?,
            CoreStatus::Running | CoreStatus::Sleeping
        ))
    }

    /// Halt, read the program counter, resume, and do it again — proving it moved.
    ///
    /// **Invasive.** Two halts and two resumes, at a moment this cannot choose. Firmware with wire
    /// timing to keep will miss it. [`Bench::status`] first; this is for when a core reports
    /// `Running` and you suspect it is spinning in a fault handler.
    pub fn prove_running(&mut self) -> Result<bool, Error> {
        let mut core = self.session.core(0)?;

        core.halt(std::time::Duration::from_millis(500))?;
        let first = core.read_core_reg::<u64>(core.program_counter())?;
        core.run()?;

        std::thread::sleep(std::time::Duration::from_millis(20));

        core.halt(std::time::Duration::from_millis(500))?;
        let second = core.read_core_reg::<u64>(core.program_counter())?;
        core.run()?;

        Ok(first != second)
    }

    /// Let a halted core run.
    ///
    /// **Attaching can leave the core halted**, depending on the probe and the debug sequence, and
    /// a harness whose whole job is watching a running board should not silently be watching a
    /// stopped one. [`Bench::status`] says which happened; this is the way back.
    pub fn resume(&mut self) -> Result<(), Error> {
        let mut core = self.session.core(0)?;
        if core.status()?.is_halted() {
            core.run()?;
        }
        Ok(())
    }

    /// Reset and let the target run.
    ///
    /// **The resume is not belt and braces.** `Core::reset` is documented as resetting and then
    /// continuing, and on an MSPM0L1306 over CMSIS-DAP it does not: the core comes back
    /// `Halted(External)` and stays there. Measured either side of the call — halted after the
    /// reset, a symbol frozen across nine seconds, and both moving again the moment `run` was
    /// issued.
    ///
    /// The failure without it is quiet, which is why this is here rather than left to the caller.
    /// Memory reads keep working on a halted core, so a target that is not executing reports its
    /// last values rather than an error, and a firmware that has stopped looks like one whose
    /// numbers happen not to be changing.
    pub fn reset(&mut self) -> Result<(), Error> {
        let mut core = self.session.core(0)?;
        core.reset()?;
        if core.status()?.is_halted() {
            core.run()?;
        }
        Ok(())
    }

    /// Reset and stop at the reset vector, so the caller can write memory before anything runs.
    ///
    /// **The window this opens is narrower than it looks.** The core is halted at the reset vector,
    /// which is *before* the startup code — so `.data` has not been copied from flash and `.bss`
    /// has not been zeroed. Anything written into either is overwritten a moment later. A value
    /// that has to survive into `main` belongs in a section the startup code does not touch;
    /// `cortex-m-rt` calls that one `.uninit`.
    ///
    /// The caller resumes with [`Bench::resume`].
    pub fn reset_and_halt(&mut self, timeout: Duration) -> Result<(), Error> {
        let mut core = self.session.core(0)?;
        core.reset_and_halt(timeout)?;
        Ok(())
    }

    /// Program an ELF onto the part, adopt it as this session's image, and let it run.
    ///
    /// **This is the operation that makes [`Verify`] unnecessary rather than the one it guards
    /// against.** Everything else here reads a part it did not put there, so the ELF and the image
    /// can disagree. After this they agree by construction, and the new symbol table replaces the
    /// old one — which is the whole of what a caller has to handle, since every name it held may
    /// have moved, changed width, or gone.
    ///
    /// # The symbol table is read before anything is erased
    ///
    /// A path that does not exist, or an ELF that does not parse, is then an error against a part
    /// that is still running. Discovering it after the erase would leave a blank part and a session
    /// with nothing to say about it.
    ///
    /// # What is not erased
    ///
    /// Flash erases by sector, and only sectors the image writes into are erased. A sector the ELF
    /// puts nothing in keeps what it held, which is what makes it safe to re-program a part whose
    /// flash also holds data written at run time — **provided that data does not share a sector
    /// with anything loadable.** Sharing one means the whole sector goes.
    ///
    /// Unwritten bytes *within* a sector that is erased are not restored, matching `probe-rs
    /// download` without `--restore-unwritten`, so this and the command line put the same thing on
    /// the part.
    ///
    /// # Cost
    ///
    /// Every written byte is read back and compared before this returns. A wrong image is otherwise
    /// silent for as long as it takes somebody to distrust a reading, and the read-back is a
    /// fraction of a second at the size a microcontroller image runs to.
    pub fn program(&mut self, elf: &Path) -> Result<(), Error> {
        self.program_watching(elf, |_| {})
    }

    /// [`Bench::program`], reporting how far through it is.
    ///
    /// `watch` is called on this thread, from inside the flash, so a caller that owns the probe on
    /// a worker thread can forward each report to an interface that is still repainting. It is the
    /// only thing that can: the erase and the read-back are one blocking call, and a front end with
    /// no reports has nothing to show for several seconds but a disabled button.
    ///
    /// # The phases are not one bar
    ///
    /// A flash erases, programs, and then reads back what it wrote, and the algorithm gives each
    /// its own total. [`Progress::fraction`] is therefore the fraction of the *current* phase, and
    /// a caller that wants one number for the whole operation is choosing weights the algorithm
    /// did not supply.
    pub fn program_watching(
        &mut self,
        elf: &Path,
        mut watch: impl FnMut(Progress),
    ) -> Result<(), Error> {
        let symbols = Symbols::load(elf)?;

        // Filled from the `AddProgressBar` events, which all arrive before any work starts.
        let mut totals: [Option<u64>; Phase::COUNT] = [None; Phase::COUNT];
        let mut done = 0u64;

        let mut options = probe_rs::flashing::DownloadOptions::default();
        options.verify = true;
        options.progress = probe_rs::flashing::FlashProgress::new(move |event| {
            report(event, &mut totals, &mut done, &mut watch);
        });
        let format = probe_rs::flashing::ElfLoader(probe_rs::flashing::ElfOptions::default());

        probe_rs::flashing::download_file_with_options(&mut self.session, elf, format, options)
            .map_err(|source| Error::Flash {
                path: elf.to_owned(),
                source: Box::new(source),
            })?;

        // Adopted only once the write succeeded. On the error path the old ELF stays, which is
        // wrong about the part — but so is every other answer, and the error says so.
        self.symbols = symbols;
        self.elf = elf.to_owned();

        self.reset()
    }

    /// Erase the chip's flash, and leave the core halted at a reset vector that holds nothing.
    ///
    /// # Which erase this is, because they are not interchangeable
    ///
    /// This drives the **flash algorithm** over the regions the target description declares, which
    /// on an MSPM0 is MAIN. It is not the vendor debug-erase sequence — that one is a boot-ROM
    /// command reached through `Session::sequence_erase_all`, it is what recovers a part whose
    /// debug port has stopped answering, and on some families it can be made to erase boot
    /// configuration as well.
    ///
    /// **The distinction is the difference between an erased application and an unreachable part.**
    /// A caller that wants the first should not be able to get the second by accident, which is why
    /// this names the narrower operation rather than taking a flag.
    ///
    /// # After this the ELF describes nothing
    ///
    /// Every symbol still resolves, because the symbol table came from the file rather than the
    /// part, and every read returns whatever erased flash reads as. That is the same hazard
    /// [`Verify`] guards at attach, arriving from the other direction — so tell somebody.
    pub fn erase(&mut self) -> Result<(), Error> {
        self.erase_watching(|_| {})
    }

    /// [`Bench::erase`], reporting how far through it is.
    ///
    /// See [`Bench::program_watching`] for why the phases are not one bar.
    pub fn erase_watching(&mut self, mut watch: impl FnMut(Progress)) -> Result<(), Error> {
        let mut totals: [Option<u64>; Phase::COUNT] = [None; Phase::COUNT];
        let mut done = 0u64;
        let mut progress = probe_rs::flashing::FlashProgress::new(move |event| {
            report(event, &mut totals, &mut done, &mut watch);
        });

        probe_rs::flashing::erase_all(&mut self.session, &mut progress, false).map_err(|source| Error::Erase {
            source: Box::new(source),
        })
    }

    /// Program bytes at an address, erasing only the sectors they land in.
    ///
    /// # What this is for, and what it is not
    ///
    /// **Placing data that is not an image**: a calibration record, a configuration block, anything
    /// a product keeps in flash and a bench has reason to set. [`Bench::program`] writes an ELF and
    /// adopts it as this session's; this writes bytes and adopts nothing, because bytes are not a
    /// symbol table.
    ///
    /// # Sectors, not bytes
    ///
    /// Flash erases by sector. `keep_unwritten_bytes` is set, so the sectors this touches are read
    /// back first and the bytes it does not write are put back as they were — which is what makes
    /// it safe to place a few words in a sector that holds something else. **It is not atomic**: a
    /// failure between the erase and the restore leaves that sector partly written.
    /// A caller that cares should check.
    ///
    /// # The ELF still describes what it described
    ///
    /// Unlike programming an image, this changes no symbol. It can, however, change what a symbol
    /// *reads* — which is the point, and worth saying because nothing about the session will look
    /// different afterwards.
    pub fn write_flash(&mut self, address: u64, bytes: &[u8]) -> Result<(), Error> {
        self.write_flash_watching(address, bytes, |_| {})
    }

    /// [`Bench::write_flash`], reporting how far through it is.
    pub fn write_flash_watching(
        &mut self,
        address: u64,
        bytes: &[u8],
        mut watch: impl FnMut(Progress),
    ) -> Result<(), Error> {
        let mut totals: [Option<u64>; Phase::COUNT] = [None; Phase::COUNT];
        let mut done = 0u64;

        let mut loader = probe_rs::flashing::FlashLoader::new(
            self.session.target().memory_map.clone(),
            probe_rs::config::TargetDescriptionSource::BuiltIn,
        );
        loader
            .add_data(address, bytes)
            .map_err(|source| Error::Erase { source: Box::new(source) })?;

        let mut options = probe_rs::flashing::DownloadOptions::default();
        options.verify = true;
        // **The whole reason this can place a few words.** Without it the sectors these bytes land
        // in are erased and only the written part comes back, so a neighbour sharing a sector is
        // lost to a write that never named it.
        options.keep_unwritten_bytes = true;
        options.progress = probe_rs::flashing::FlashProgress::new(move |event| {
            report(event, &mut totals, &mut done, &mut watch);
        });

        loader
            .commit(&mut self.session, options)
            .map_err(|source| Error::Erase { source: Box::new(source) })
    }

    /// Read one value by symbol name, while the core runs.
    ///
    /// The ELF's recorded width is checked against `T` first, so asking for the wrong type is an
    /// error rather than three bytes of a neighbour.
    pub fn peek<T: Value>(&mut self, name: &str) -> Result<T, Error> {
        let symbol = self.symbols.get(name)?;
        check(name, symbol, T::WIDTH, T::NAME)?;
        let mut core = self.session.core(0)?;
        T::read(&mut core, symbol.address)
    }

    /// Write one value by symbol name, while the core runs, and read it back.
    ///
    /// **The read-back is not optional here, and it is not validating the value.** Nothing in this
    /// crate has an opinion about whether a poked number is sensible — a sweep exists to find where
    /// the logic stops holding, so an out-of-range value is the experiment. What the read-back
    /// catches is the write not landing at all: a symbol in flash, or an address the core is
    /// holding. Without it a sweep silently repeats one point and draws a flat curve, which is a
    /// working-looking result and the hardest kind of wrong.
    ///
    /// Only for locations the firmware does not itself write. For those, see
    /// [`Bench::poke_unchecked`].
    pub fn poke<T: Value>(&mut self, name: &str, value: T) -> Result<(), Error> {
        self.poke_unchecked(name, value)?;
        let read: T = self.peek(name)?;
        if read.as_u64() != value.as_u64() {
            return Err(Error::PokeDidNotStick {
                name: name.to_owned(),
                wrote: value.as_u64(),
                read: read.as_u64(),
            });
        }
        Ok(())
    }

    /// Write without reading back, for a location the firmware also writes.
    ///
    /// A read-back there is a race rather than a check, and it would report a failure whenever the
    /// firmware happened to win it.
    pub fn poke_unchecked<T: Value>(&mut self, name: &str, value: T) -> Result<(), Error> {
        let symbol = self.symbols.get(name)?;
        check(name, symbol, T::WIDTH, T::NAME)?;
        let mut core = self.session.core(0)?;
        value.write(&mut core, symbol.address)
    }

    /// Read a symbol whose width is not one of the scalar types — an array, or a struct.
    ///
    /// No width check, because there is no type to check against. The ELF's recorded size is what
    /// a caller should ask for, and [`Symbols::get`] is how to find it.
    pub fn peek_bytes(&mut self, name: &str, len: usize) -> Result<Vec<u8>, Error> {
        let symbol = self.symbols.get(name)?;
        let mut out = vec![0u8; len];
        let mut core = self.session.core(0)?;
        core.read(symbol.address, &mut out)?;
        Ok(out)
    }

    /// Whether a symbol is in this image at all.
    ///
    /// For state a HAL exports only under some feature, where absent and zero mean different
    /// things and reporting the second for the first is a wrong answer rather than a missing one.
    pub fn has(&self, name: &str) -> bool {
        self.symbols.get(name).is_ok()
    }

    /// Read a word at a raw address, for a peripheral register rather than a symbol.
    pub fn read_u32(&mut self, address: u64) -> Result<u32, Error> {
        let mut core = self.session.core(0)?;
        Ok(core.read_word_32(address)?)
    }

    /// Write a word at a raw address.
    ///
    /// **No read-back here, unlike [`Bench::poke`].** A peripheral register is not memory: many are
    /// write-one-to-set or write-one-to-clear, and reading one back after writing it compares two
    /// different things and reports a failure that is not one.
    pub fn write_u32(&mut self, address: u64, value: u32) -> Result<(), Error> {
        let mut core = self.session.core(0)?;
        Ok(core.write_word_32(address, value)?)
    }

    /// Follow this target's log.
    ///
    /// Here rather than on the reader because the control block's address and the session both
    /// come out of this struct, and a caller cannot borrow one while passing the other.
    pub fn logs(&mut self, channel: usize) -> Result<log::Reader, Error> {
        let elf = self.elf.clone();
        let region = log::region(&self.symbols)?;
        log::Reader::attach(&mut self.session, region, &elf, channel)
    }

    /// The session, for anything this does not wrap yet.
    pub fn session(&mut self) -> &mut Session {
        &mut self.session
    }

    /// Leave the core halted when this is dropped, instead of running.
    ///
    /// For a caller that halted the core deliberately and wants it to stay that way — reading a
    /// consistent snapshot of something the firmware is writing, or handing the part to a debugger.
    pub fn leave_halted(&mut self) {
        self.resume_on_drop = false;
    }
}

/// Resume before detaching.
///
/// **Necessary and, on MSPM0, not sufficient — which is worth knowing before relying on it.**
/// Attaching leaves the core halted, so a session that never resumes and then exits leaves it
/// halted too. This covers that.
///
/// It does not cover everything. Measured on an L1306: a counter the firmware bumps every few
/// seconds reads zero immediately after a tool that resumed the core exits, and zero again twelve
/// seconds later — so the part is not running between sessions even with this in place. It runs
/// perfectly *during* one, which is what a sweep needs, so the effect is on walking away rather
/// than on measuring.
///
/// The cause is not established. probe-rs's own teardown is not an obvious candidate: `Session`'s
/// drop clears breakpoints and calls `debug_core_stop`, and that sequence tears debug down and
/// hands low-power control back without resetting. **Do not attach a story to it.** The workaround
/// is one command — `probe-rs reset` — and the question is written down rather than guessed at.
///
/// Best-effort, because a destructor has nowhere to return an error to. A failure here means the
/// session was already broken, which the caller has heard about by another route.
impl Drop for Bench {
    fn drop(&mut self) {
        if !self.resume_on_drop {
            return;
        }
        let resumed = self
            .session
            .core(0)
            .and_then(|mut core| {
                if core.status()?.is_halted() {
                    core.run()?;
                }
                Ok(())
            });
        if let Err(e) = resumed {
            tracing::warn!("could not resume the core on detach, so the board is stopped: {e}");
        }
    }
}

/// The ELF's width and alignment against the type asked for.
fn check(name: &str, symbol: Symbol, wanted: u64, type_name: &'static str) -> Result<(), Error> {
    // A zero size means the producer did not record one, which is common for assembly symbols. It
    // is not evidence of a mismatch, so it is not treated as one.
    if symbol.size != 0 && symbol.size != wanted {
        return Err(Error::WidthMismatch {
            name: name.to_owned(),
            actual: symbol.size,
            wanted,
            type_name,
        });
    }
    if !symbol.address.is_multiple_of(wanted) {
        return Err(Error::Misaligned {
            name: name.to_owned(),
            address: symbol.address,
            alignment: wanted,
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{Phase, Progress};

    fn at(done: u64, total: Option<u64>) -> Progress {
        Progress { phase: Phase::Program, done, total }
    }

    #[test]
    fn a_fraction_needs_a_total() {
        assert_eq!(at(512, None).fraction(), None);
        assert_eq!(at(512, Some(1024)).fraction(), Some(0.5));
    }

    /// A phase with nothing to do is finished, not at the start of itself. A bar rendered from
    /// zero over zero would sit empty for the whole of an operation that never runs.
    #[test]
    fn an_empty_phase_is_complete() {
        assert_eq!(at(0, Some(0)).fraction(), Some(1.0));
    }

    /// The algorithm rounds a page to its own granularity, so the reports can add up past the
    /// total it published. A bar is clamped rather than allowed past its end.
    #[test]
    fn overshooting_the_total_does_not_overshoot_the_bar() {
        assert_eq!(at(2048, Some(1024)).fraction(), Some(1.0));
    }

    #[test]
    fn every_phase_has_its_own_slot() {
        let all = [Phase::Fill, Phase::Erase, Phase::Program, Phase::Verify, Phase::Ram];
        assert_eq!(all.len(), Phase::COUNT);
        let mut seen: Vec<usize> = all.iter().map(|p| p.index()).collect();
        seen.sort_unstable();
        seen.dedup();
        assert_eq!(seen.len(), Phase::COUNT, "two phases share a slot, so one overwrites the other's total");
        assert!(seen.iter().all(|&i| i < Phase::COUNT), "a slot is outside the table");
    }
}
