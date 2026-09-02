//! The ELF's symbol table, which is the whole schema.
//!
//! There is no protocol between host and target here, and no generated header kept in step by hand.
//! A parameter is a `static` in the firmware; the harness looks its name up and writes it. The
//! symbol table cannot drift from the image because it *is* the image.

use std::collections::HashMap;
use std::path::Path;

use object::{Object, ObjectSymbol};

use crate::Error;

/// One symbol: where it lives and how big the linker says it is.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Symbol {
    /// Target address, with the Thumb bit already cleared.
    pub address: u64,
    /// Size in bytes, from the ELF. Zero when the producer did not record one.
    pub size: u64,
}

/// Every named symbol in an ELF, by name.
pub struct Symbols {
    by_name: HashMap<String, Symbol>,
}

impl Symbols {
    /// Read an ELF and index every symbol that has a name.
    pub fn load(elf: &Path) -> Result<Self, Error> {
        let bytes = std::fs::read(elf).map_err(|source| Error::ElfRead {
            path: elf.to_path_buf(),
            source,
        })?;
        let file = object::File::parse(&*bytes).map_err(|source| Error::ElfParse {
            path: elf.to_path_buf(),
            source,
        })?;

        let mut by_name = HashMap::new();
        for symbol in file.symbols() {
            let Ok(name) = symbol.name() else { continue };
            if name.is_empty() {
                continue;
            }
            // ARM mapping symbols share addresses with real ones and would shadow them.
            if name.starts_with('$') {
                continue;
            }
            // The Thumb bit is set on function symbols and is not part of the address. It is
            // cleared here rather than at every use, because a caller reading a `static` should
            // never have to think about it and a caller calling a function needs it set back —
            // which is that caller's business, and is documented where it arises.
            by_name.insert(
                name.to_owned(),
                Symbol {
                    address: symbol.address() & !1,
                    size: symbol.size(),
                },
            );
        }

        Ok(Self { by_name })
    }

    /// Look one up.
    pub fn get(&self, name: &str) -> Result<Symbol, Error> {
        self.by_name.get(name).copied().ok_or_else(|| Error::NoSuchSymbol {
            name: name.to_owned(),
            near: self.nearest(name),
        })
    }

    /// Every symbol whose name contains `needle`, sorted. For a CLI that lists what is settable.
    pub fn containing(&self, needle: &str) -> Vec<(&str, Symbol)> {
        let mut found: Vec<_> = self
            .by_name
            .iter()
            .filter(|(name, _)| name.contains(needle))
            .map(|(name, symbol)| (name.as_str(), *symbol))
            .collect();
        found.sort_by_key(|(name, _)| *name);
        found
    }

    pub fn len(&self) -> usize {
        self.by_name.len()
    }

    pub fn is_empty(&self) -> bool {
        self.by_name.is_empty()
    }

    /// A few names sharing the longest prefix with `name`, for the error message.
    ///
    /// A typo in a symbol name is the most likely way to reach `NoSuchSymbol`, and a bare "not
    /// found" against a table of several thousand is the least useful thing to say about one.
    fn nearest(&self, name: &str) -> Vec<String> {
        let prefix_len = |candidate: &str| name.bytes().zip(candidate.bytes()).take_while(|(a, b)| a == b).count();
        let best = self.by_name.keys().map(|k| prefix_len(k)).max().unwrap_or(0);
        // Nothing in common is not a suggestion, it is noise.
        if best < 3 {
            return Vec::new();
        }
        let mut near: Vec<String> = self
            .by_name
            .keys()
            .filter(|k| prefix_len(k) == best)
            .take(8)
            .cloned()
            .collect();
        near.sort();
        near
    }
}
