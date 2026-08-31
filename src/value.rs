//! The scalar types a symbol can be read or written as.
//!
//! Deliberately only the ones a debugger can move in a single bus transaction. A wider or
//! compound type would need a byte-slice read and a layout the host and target agree on, which is
//! the schema this crate exists to avoid having.

use probe_rs::{Core, MemoryInterface};

use crate::Error;

/// A scalar that can be read from or written to target memory by address.
pub trait Value: Copy + Sized {
    /// Bytes on the target, checked against what the ELF records for the symbol.
    const WIDTH: u64;
    /// For the error message, so a width mismatch names the type the caller asked for.
    const NAME: &'static str;

    fn read(core: &mut Core<'_>, address: u64) -> Result<Self, Error>;
    fn write(self, core: &mut Core<'_>, address: u64) -> Result<(), Error>;

    /// Widened, for comparing a read-back against what was written.
    ///
    /// Signed types are cast through their unsigned counterpart first, so the comparison is over
    /// the bits that were actually moved rather than over a sign-extended interpretation of them.
    fn as_u64(self) -> u64;
}

macro_rules! scalar {
    ($ty:ty, $unsigned:ty, $width:expr, $read:ident, $write:ident) => {
        impl Value for $ty {
            const WIDTH: u64 = $width;
            const NAME: &'static str = stringify!($ty);

            fn read(core: &mut Core<'_>, address: u64) -> Result<Self, Error> {
                Ok(core.$read(address)? as $ty)
            }

            fn write(self, core: &mut Core<'_>, address: u64) -> Result<(), Error> {
                Ok(core.$write(address, self as $unsigned)?)
            }

            fn as_u64(self) -> u64 {
                self as $unsigned as u64
            }
        }
    };
}

scalar!(u8, u8, 1, read_word_8, write_word_8);
scalar!(u16, u16, 2, read_word_16, write_word_16);
scalar!(u32, u32, 4, read_word_32, write_word_32);
scalar!(u64, u64, 8, read_word_64, write_word_64);
scalar!(i8, u8, 1, read_word_8, write_word_8);
scalar!(i16, u16, 2, read_word_16, write_word_16);
scalar!(i32, u32, 4, read_word_32, write_word_32);
scalar!(i64, u64, 8, read_word_64, write_word_64);

/// `bool` is one byte and any non-zero is true, matching how Rust lays it out.
///
/// Writing normalises to 0 or 1 rather than passing a caller's byte through: a `bool` holding
/// anything else is undefined behaviour on the target, and this crate is the thing that would
/// create one.
impl Value for bool {
    const WIDTH: u64 = 1;
    const NAME: &'static str = "bool";

    fn read(core: &mut Core<'_>, address: u64) -> Result<Self, Error> {
        Ok(core.read_word_8(address)? != 0)
    }

    fn write(self, core: &mut Core<'_>, address: u64) -> Result<(), Error> {
        Ok(core.write_word_8(address, u8::from(self))?)
    }

    fn as_u64(self) -> u64 {
        u64::from(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The widths are the ones the checker compares against the ELF, so a wrong one here is a
    /// silent partial write rather than a failure.
    #[test]
    fn widths_match_the_rust_types() {
        assert_eq!(u8::WIDTH as usize, size_of::<u8>());
        assert_eq!(u16::WIDTH as usize, size_of::<u16>());
        assert_eq!(u32::WIDTH as usize, size_of::<u32>());
        assert_eq!(u64::WIDTH as usize, size_of::<u64>());
        assert_eq!(i8::WIDTH as usize, size_of::<i8>());
        assert_eq!(i16::WIDTH as usize, size_of::<i16>());
        assert_eq!(i32::WIDTH as usize, size_of::<i32>());
        assert_eq!(i64::WIDTH as usize, size_of::<i64>());
        assert_eq!(bool::WIDTH as usize, size_of::<bool>());
    }

    /// A negative value and its unsigned bit pattern have to compare equal, or every read-back of
    /// a poked negative number reports as not having stuck.
    #[test]
    fn a_signed_value_widens_by_its_bits() {
        assert_eq!((-1i8).as_u64(), 0xff);
        assert_eq!((-1i16).as_u64(), 0xffff);
        assert_eq!((-1i32).as_u64(), 0xffff_ffff);
        assert_eq!(i32::MIN.as_u64(), 0x8000_0000);
        // And it is not a sign extension, which is the failure this shape prevents.
        assert_ne!((-1i32).as_u64(), u64::MAX);
    }
}
