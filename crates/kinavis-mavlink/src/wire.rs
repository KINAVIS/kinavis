//! Little-endian fields of a payload: zeros read beyond its end, as MAVLink 2
//! drops trailing zero bytes; writes beyond its end are dropped.

/// Reader over a payload.
pub(crate) struct Reader<'a> {
    bytes: &'a [u8],
    at: usize,
}

macro_rules! read {
    ($name:ident, $type:ty) => {
        pub(crate) fn $name(&mut self) -> $type {
            let mut raw = [0u8; core::mem::size_of::<$type>()];
            for (slot, byte) in raw.iter_mut().zip(self.bytes.iter().skip(self.at)) {
                *slot = *byte;
            }
            self.at = self.at.saturating_add(raw.len());
            <$type>::from_le_bytes(raw)
        }
    };
}

#[allow(dead_code)]
impl<'a> Reader<'a> {
    pub(crate) const fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, at: 0 }
    }

    read!(u8, u8);
    read!(i8, i8);
    read!(u16, u16);
    read!(i16, i16);
    read!(u32, u32);
    read!(i32, i32);
    read!(u64, u64);
    read!(i64, i64);
    read!(f32, f32);
    read!(f64, f64);
}

/// Writer into a payload.
pub(crate) struct Writer<'a> {
    bytes: &'a mut [u8],
    at: usize,
}

macro_rules! write {
    ($name:ident, $type:ty) => {
        pub(crate) fn $name(&mut self, value: $type) {
            let raw = value.to_le_bytes();
            for (slot, byte) in self.bytes.iter_mut().skip(self.at).zip(raw) {
                *slot = byte;
            }
            self.at = self.at.saturating_add(raw.len());
        }
    };
}

#[allow(dead_code)]
impl<'a> Writer<'a> {
    pub(crate) fn new(bytes: &'a mut [u8]) -> Self {
        Self { bytes, at: 0 }
    }

    write!(u8, u8);
    write!(i8, i8);
    write!(u16, u16);
    write!(i16, i16);
    write!(u32, u32);
    write!(i32, i32);
    write!(u64, u64);
    write!(i64, i64);
    write!(f32, f32);
    write!(f64, f64);
}

/// X.25 (CRC-16/MCRF4XX) checksum of MAVLink, from its seed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Crc(u16);

impl Crc {
    pub(crate) const fn new() -> Self {
        Self(0xFFFF)
    }

    pub(crate) fn add(&mut self, bytes: &[u8]) {
        for byte in bytes {
            let [low, _] = self.0.to_le_bytes();
            let mut tmp = byte ^ low;
            tmp ^= tmp << 4;
            let tmp = u16::from(tmp);
            self.0 = (self.0 >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
        }
    }

    pub(crate) const fn value(self) -> u16 {
        self.0
    }
}
