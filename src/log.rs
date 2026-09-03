//! RTT, decoded to structured lines rather than to text.
//!
//! # Why not text
//!
//! What the target sends is not coloured text. `defmt-decoder` produces frames carrying a level, a
//! timestamp, a message and an index that maps to a file and line; the colours in a terminal are
//! applied by the host *from that structure*. Flattening to ANSI and parsing it back would destroy
//! everything worth having:
//!
//! - the timestamp stays a number, so it can be shown as a delta or aligned with a sweep's sample
//!   index rather than re-parsed out of a rendered string;
//! - filtering by level or module is a predicate over fields rather than a grep over escapes;
//! - `file:line` is already on the line, so an editor can be opened from it;
//! - colour comes from the viewer's theme rather than from a sixteen-colour palette that assumes a
//!   dark terminal.
//!
//! So this owns the decode and hands out [`Line`], which owns its strings. That conversion is
//! forced anyway: a decoder frame borrows the string table and cannot cross a channel.
//!
//! # Raw bytes still have a lane
//!
//! Two things need one: a non-defmt RTT channel, and a production image, which has no defmt in it
//! at all. [`Reader::attach`] falls back to raw when no table parses out of the ELF, and says which
//! it chose.

use std::path::Path;

use defmt_decoder::{Location, Table};
use probe_rs::rtt::{Rtt, ScanRegion, UpChannel};
use probe_rs::{Core, Session};

use crate::Error;

/// The RTT control block's symbol, as every implementation of it names the thing.
pub const CONTROL_BLOCK: &str = "_SEGGER_RTT";

/// Where the control block is, from the ELF.
///
/// **The ELF says where it is, so do not scan for it.** A RAM scan looks for a magic string and
/// will find one in an image that has no RTT at all — measured, on a production build with
/// `defmt-rtt` compiled out: it attached "successfully" and then produced nothing, forever, with
/// no error to explain it. The symbol is the authority here as it is everywhere else in this
/// crate, and its absence is a clear answer rather than a slow one.
pub fn region(symbols: &crate::Symbols) -> Result<ScanRegion, Error> {
    match symbols.get(CONTROL_BLOCK) {
        Ok(symbol) => Ok(ScanRegion::Exact(symbol.address)),
        Err(_) => Err(Error::NoRtt {
            detail: format!("this image has no `{CONTROL_BLOCK}` symbol, so it was built without RTT"),
        }),
    }
}

/// One decoded log line, owning everything it carries.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Line {
    /// `None` on a raw channel, or on a frame the target sent without one.
    pub level: Option<String>,
    /// The device's own timestamp, as the firmware formats it. `None` if the image has none.
    pub timestamp: Option<String>,
    pub message: String,
    /// Where in the source it came from, when the ELF carries locations.
    pub location: Option<Where>,
}

/// A decoded frame's source position.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Where {
    pub file: String,
    pub line: u32,
    /// **Carried and not yet read by anything here**, which is deliberate rather than an
    /// oversight: this module's own opening says filtering by module should be a predicate over a
    /// field rather than a grep over escapes, and this is that field. A consumer that filters by it
    /// has not been written; dropping it would mean re-deriving it from the ELF when one is.
    pub module: String,
}

/// How a channel is being read.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Format {
    /// Frames decoded against the ELF's string table.
    Defmt,
    /// Bytes, split into lines. What a production image sends, and what a non-defmt channel is.
    Raw,
}

/// A live view of one RTT up-channel.
pub struct Reader {
    channel: UpChannel,
    format: Format,
    /// The decoder, kept alive across polls so a frame split over two reads is not lost.
    ///
    /// **The `Table` behind it is leaked deliberately.** `new_stream_decoder` borrows the table,
    /// and a struct cannot hold both without a self-reference; the alternative is rebuilding the
    /// decoder every poll, which throws away the partial frame at the end of each read and
    /// silently drops a line whenever one straddles a poll boundary. One table for the life of the
    /// process is the cheaper mistake, and in a tool it is not a mistake at all.
    stream: Option<Box<dyn defmt_decoder::StreamDecoder + Send + Sync + 'static>>,
    has_timestamp: bool,
    locations: Option<defmt_decoder::Locations>,
    /// Bytes received and not yet split into whole lines. Raw channels only; the defmt decoder
    /// keeps its own.
    pending: Vec<u8>,
    buffer: Vec<u8>,
}

impl Reader {
    /// Find the control block and take channel `index`, decoding as defmt if the ELF says so.
    ///
    /// **The version check is on the wire format, not the crate version.** `Table::parse` compares
    /// what the ELF declares against what this decoder speaks, and failing here with a clear
    /// message beats decoding garbage into plausible-looking lines.
    pub fn attach(session: &mut Session, region: ScanRegion, elf: &Path, index: usize) -> Result<Self, Error> {
        let bytes = std::fs::read(elf).map_err(|source| Error::ElfRead {
            path: elf.to_path_buf(),
            source,
        })?;

        let table = Table::parse(&bytes).map_err(|source| Error::DefmtTable {
            path: elf.to_path_buf(),
            source,
        })?;
        let locations = table
            .as_ref()
            .and_then(|t| t.get_locations(&bytes).ok())
            .filter(|l| !l.is_empty());
        let format = if table.is_some() { Format::Defmt } else { Format::Raw };
        let has_timestamp = table.as_ref().is_some_and(Table::has_timestamp);
        let stream = table
            .map(|t| Box::leak(Box::new(t)) as &'static Table)
            .map(Table::new_stream_decoder);

        let mut core = crate::acquire(session)?;
        let rtt = Rtt::attach_region(&mut core, &region).map_err(|e| Error::NoRtt {
            detail: format!("at the address `{CONTROL_BLOCK}` names: {e}"),
        })?;

        let mut channels = rtt.up_channels;
        if index >= channels.len() {
            return Err(Error::NoSuchChannel {
                index,
                len: channels.len(),
            });
        }
        let channel = channels.remove(index);

        Ok(Self {
            channel,
            format,
            stream,
            has_timestamp,
            locations,
            pending: Vec::new(),
            buffer: vec![0u8; 4096],
        })
    }

    pub fn format(&self) -> Format {
        self.format
    }

    /// Whether the image carries a device timestamp.
    ///
    /// Without one the host would stamp arrival instead, which includes the RTT buffer and the
    /// poll period — the wrong clock for anything being compared against a deadline.
    pub fn has_timestamp(&self) -> bool {
        self.has_timestamp
    }

    /// Drain whatever the target has produced since the last call.
    ///
    /// Returns an empty vector when there is nothing, which is the common case at a poll rate
    /// higher than the target logs at.
    pub fn poll(&mut self, core: &mut Core<'_>) -> Result<Vec<Line>, Error> {
        let _span = tracing::debug_span!("log_poll").entered();
        let read = self.channel.read(core, &mut self.buffer)?;

        match self.format {
            Format::Defmt => {
                if read == 0 {
                    return Ok(Vec::new());
                }
                self.decode_defmt(read)
            }
            Format::Raw => {
                if read == 0 && self.pending.is_empty() {
                    return Ok(Vec::new());
                }
                let chunk = self.buffer[..read].to_vec();
                self.pending.extend_from_slice(&chunk);
                Ok(self.split_raw())
            }
        }
    }

    fn decode_defmt(&mut self, read: usize) -> Result<Vec<Line>, Error> {
        let Some(stream) = &mut self.stream else {
            return Ok(Vec::new());
        };
        stream.received(&self.buffer[..read]);

        let mut out = Vec::new();
        loop {
            match stream.decode() {
                Ok(frame) => {
                    let location = self.locations.as_ref().and_then(|l| l.get(&frame.index())).map(
                        |Location { file, line, module, .. }| Where {
                            file: file.display().to_string(),
                            line: *line as u32,
                            module: module.clone(),
                        },
                    );
                    out.push(Line {
                        level: frame.level().map(|l| format!("{l:?}").to_uppercase()),
                        timestamp: frame.display_timestamp().map(|t| t.to_string()),
                        message: frame.display_message().to_string(),
                        location,
                    });
                }
                // A partial frame is the normal state between polls, not an error: the decoder
                // keeps it and the rest arrives next time. That is the whole reason the decoder
                // outlives the poll.
                Err(defmt_decoder::DecodeError::UnexpectedEof) => break,
                Err(defmt_decoder::DecodeError::Malformed) => return Err(Error::MalformedDefmt),
            }
        }
        Ok(out)
    }

    fn split_raw(&mut self) -> Vec<Line> {
        let mut out = Vec::new();
        while let Some(at) = self.pending.iter().position(|&b| b == b'\n') {
            let line: Vec<u8> = self.pending.drain(..=at).collect();
            let text = String::from_utf8_lossy(&line).trim_end().to_owned();
            out.push(Line {
                level: None,
                timestamp: None,
                message: text,
                location: None,
            });
        }
        out
    }
}
