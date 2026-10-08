//! Little-endian fields of a payload, by offset; out of range is `None`.

pub(crate) fn u8_at(payload: &[u8], offset: usize) -> Option<u8> {
    payload.get(offset).copied()
}

pub(crate) fn i8_at(payload: &[u8], offset: usize) -> Option<i8> {
    u8_at(payload, offset).map(|byte| i8::from_le_bytes([byte]))
}

fn bytes<const K: usize>(payload: &[u8], offset: usize) -> Option<[u8; K]> {
    payload
        .get(offset..offset.checked_add(K)?)
        .and_then(|slice| <[u8; K]>::try_from(slice).ok())
}

pub(crate) fn u16_at(payload: &[u8], offset: usize) -> Option<u16> {
    bytes(payload, offset).map(u16::from_le_bytes)
}

pub(crate) fn i16_at(payload: &[u8], offset: usize) -> Option<i16> {
    bytes(payload, offset).map(i16::from_le_bytes)
}

pub(crate) fn u32_at(payload: &[u8], offset: usize) -> Option<u32> {
    bytes(payload, offset).map(u32::from_le_bytes)
}

pub(crate) fn i32_at(payload: &[u8], offset: usize) -> Option<i32> {
    bytes(payload, offset).map(i32::from_le_bytes)
}

pub(crate) fn f64_at(payload: &[u8], offset: usize) -> Option<f64> {
    bytes(payload, offset).map(f64::from_le_bytes)
}
