//! Checking that the ELF describes the image actually on the part.
//!
//! # Why this exists
//!
//! The rest of this crate rests on one claim: the ELF's symbol table is the schema, and it cannot
//! drift from the image because it *is* the image. That is true of the ELF and **false of the ELF
//! against whatever is flashed**. A stale ELF resolves every symbol, returns plausible addresses,
//! and reads whatever the previous image left at them.
//!
//! Found the first time this crate was pointed at a board: a tick count that should have been 2
//! read as `0x8702007f`, from an ELF one build out of date. Nothing about that reading said it was
//! wrong. A sweep taken through a stale ELF would have produced a curve.
//!
//! So verification is on by default and opting out is explicit.

use std::path::Path;

use object::Endianness;
use object::read::elf::{ElfFile32, ProgramHeader};
use probe_rs::{Core, MemoryInterface};

use crate::Error;

/// How much of the image to check against the target when attaching.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Verify {
    /// Every loadable byte. Exact, names the first address that differs, and a small image costs
    /// well under a second over SWD.
    #[default]
    Full,
    /// A window at the start, the middle and the end of each loadable segment.
    ///
    /// Cheap, and it catches a different build rather than a corrupted one — which is the failure
    /// that actually happens. Two builds of the same source differ in their vector table almost
    /// never and in their body almost always.
    Sampled,
    /// Trust the ELF.
    ///
    /// For attaching to a board whose image you do not have, or one you have deliberately poked
    /// somewhere the ELF describes.
    Skip,
}

/// One loadable chunk: where it goes on the target, and what should be there.
struct Load<'a> {
    address: u64,
    bytes: &'a [u8],
}

/// Compare the ELF's loadable content against the target.
pub fn verify(core: &mut Core<'_>, elf: &Path, how: Verify) -> Result<(), Error> {
    if how == Verify::Skip {
        return Ok(());
    }

    let bytes = std::fs::read(elf).map_err(|source| Error::ElfRead {
        path: elf.to_path_buf(),
        source,
    })?;
    let file = ElfFile32::<Endianness>::parse(&*bytes).map_err(|source| Error::ElfParse {
        path: elf.to_path_buf(),
        source,
    })?;

    // **`p_paddr`, not `p_vaddr`.** `.data`'s initialiser lives in flash and its variables live in
    // RAM, so the two differ for exactly the segment whose RAM copy the firmware has been writing
    // to since boot. Comparing at the virtual address would read those variables and report a
    // mismatch on every running board.
    let endian = file.endian();
    let mut loads = Vec::new();
    for header in file.elf_program_headers() {
        if header.p_type(endian) != object::elf::PT_LOAD {
            continue;
        }
        let Ok(Some(data)) = header.data(endian, &*bytes).map(Some) else {
            continue;
        };
        if data.is_empty() {
            continue;
        }
        loads.push(Load {
            address: u64::from(header.p_paddr(endian)),
            bytes: data,
        });
    }

    if loads.is_empty() {
        return Err(Error::NothingLoadable {
            path: elf.to_path_buf(),
        });
    }

    for load in &loads {
        match how {
            Verify::Full => compare(core, load.address, load.bytes, 0, elf)?,
            Verify::Sampled => {
                for (offset, len) in windows(load.bytes.len()) {
                    compare(
                        core,
                        load.address + offset as u64,
                        &load.bytes[offset..offset + len],
                        offset,
                        elf,
                    )?;
                }
            }
            Verify::Skip => unreachable!("returned above"),
        }
    }

    Ok(())
}

/// Start, middle and end of a segment, clamped and deduplicated.
fn windows(len: usize) -> Vec<(usize, usize)> {
    const WINDOW: usize = 64;
    if len <= WINDOW * 3 {
        return vec![(0, len)];
    }
    let mut spans = vec![(0, WINDOW), (len / 2 - WINDOW / 2, WINDOW), (len - WINDOW, WINDOW)];
    spans.dedup();
    spans
}

fn compare(core: &mut Core<'_>, address: u64, expected: &[u8], offset: usize, elf: &Path) -> Result<(), Error> {
    let mut actual = vec![0u8; expected.len()];
    core.read(address, &mut actual)?;

    if let Some(at) = actual.iter().zip(expected).position(|(a, b)| a != b) {
        return Err(Error::ImageMismatch {
            path: elf.to_path_buf(),
            address: address + at as u64,
            expected: expected[at],
            actual: actual[at],
            offset: offset + at,
        });
    }
    Ok(())
}
