//! Fixed-capacity storage; nothing built on the kernel needs an allocator.
//!
//! Bridge navigation computers are often heapless microcontrollers, and
//! everything the navigation crates store has a natural bound (swing nodes
//! every few degrees, a few dozen waypoints, a short error excerpt): fixed
//! capacity, checked on insertion.
//!
//! Public so that aggregates in `kinavis` (deviation table, route) and adapters
//! with the same constraint share it.
//!
//! No `unsafe`: [`Inline`] holds a fully initialised array and a live length;
//! the unused tail is never visible through [`Inline::as_slice`].

use core::fmt;
use core::ops::Deref;

/// Vector of at most `N` items, stored inline.
///
/// Capacity is part of the type; `push` and `insert` report a full store
/// instead of growing or truncating. Items are read through the slice it
/// dereferences to.
///
/// `T: Copy` because the array is pre-filled with a fill value, which avoids
/// `unsafe`.
#[derive(Clone, Copy)]
pub struct Inline<T: Copy, const N: usize> {
    items: [T; N],
    len: usize,
}

impl<T: Copy, const N: usize> Inline<T, N> {
    /// Empty store; `fill` only occupies the unused tail.
    #[must_use]
    pub const fn new(fill: T) -> Self {
        Self {
            items: [fill; N],
            len: 0,
        }
    }

    /// Appends an item.
    ///
    /// # Errors
    ///
    /// [`Full`] if `N` items are held; the store is unchanged.
    pub fn push(&mut self, item: T) -> Result<(), Full> {
        // Read the length once and compare before storing, so the compiler sees
        // the increment cannot wrap; re-reading after the store would lose that
        // proof (possible aliasing).
        let len = self.len;
        if len >= N {
            return Err(Full { capacity: N });
        }
        let slot = self.items.get_mut(len).ok_or(Full { capacity: N })?;
        *slot = item;
        self.len = len + 1;
        Ok(())
    }

    /// Inserts at `index`, shifting later items; an index past the end appends.
    ///
    /// # Errors
    ///
    /// [`Full`] if `N` items are held; the store is unchanged.
    pub fn insert(&mut self, index: usize, item: T) -> Result<(), Full> {
        // Read once, as in `push`, so every step is visibly bounded by `N`.
        let len = self.len;
        if len >= N {
            return Err(Full { capacity: N });
        }
        let index = index.min(len);
        // Iterate backwards so nothing is overwritten.
        let mut source = len;
        while source > index {
            source -= 1;
            let value = *self.items.get(source).ok_or(Full { capacity: N })?;
            *self.items.get_mut(source + 1).ok_or(Full { capacity: N })? = value;
        }
        *self.items.get_mut(index).ok_or(Full { capacity: N })? = item;
        self.len = len + 1;
        Ok(())
    }

    /// Removes and returns the item at `index`, closing the gap; `None` past
    /// the end, store unchanged.
    pub fn remove(&mut self, index: usize) -> Option<T> {
        let len = self.len;
        if index >= len {
            return None;
        }
        let removed = *self.items.get(index)?;
        // Shift the tail down; `index < len <= N` bounds every step.
        let mut at = index;
        while at + 1 < len {
            let next = *self.items.get(at + 1)?;
            *self.items.get_mut(at)? = next;
            at += 1;
        }
        self.len = len - 1;
        Some(removed)
    }

    /// Items.
    #[must_use]
    pub fn as_slice(&self) -> &[T] {
        self.items.get(..self.len).unwrap_or(&[])
    }

    /// Items, mutable.
    #[must_use]
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        self.items.get_mut(..self.len).unwrap_or(&mut [])
    }

    /// Length.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Whether empty.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Capacity.
    #[must_use]
    pub const fn capacity() -> usize {
        N
    }
}

impl<T: Copy, const N: usize> Deref for Inline<T, N> {
    type Target = [T];

    fn deref(&self) -> &[T] {
        self.as_slice()
    }
}

impl<T: Copy + fmt::Debug, const N: usize> fmt::Debug for Inline<T, N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_list().entries(self.as_slice()).finish()
    }
}

impl<T: Copy + PartialEq, const N: usize> PartialEq for Inline<T, N> {
    /// Compares live items only.
    fn eq(&self, other: &Self) -> bool {
        self.as_slice() == other.as_slice()
    }
}

/// Store full.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Full {
    /// Capacity.
    pub capacity: usize,
}

impl fmt::Display for Full {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "no room left in a store of {} items", self.capacity)
    }
}

impl core::error::Error for Full {}

/// Short inline string.
///
/// Used for error excerpts. Input longer than `N` bytes is truncated on a
/// character boundary with an ellipsis.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct InlineStr<const N: usize> {
    bytes: [u8; N],
    len: usize,
}

impl<const N: usize> InlineStr<N> {
    /// Copies as much of `text` as fits, truncating on a character boundary.
    #[must_use]
    pub fn new(text: &str) -> Self {
        let mut bytes = [0_u8; N];
        let mut len = 0;
        for (index, character) in text.char_indices() {
            // A `str` byte index plus at most four cannot wrap, but the
            // compiler does not know `str` length is bounded.
            let end = index.saturating_add(character.len_utf8());
            if end > N {
                break;
            }
            len = end;
        }
        for (slot, byte) in bytes.iter_mut().zip(text.as_bytes().iter().take(len)) {
            *slot = *byte;
        }
        Self { bytes, len }
    }

    /// Text, possibly truncated.
    #[must_use]
    pub fn as_str(&self) -> &str {
        self.bytes
            .get(..self.len)
            .and_then(|bytes| core::str::from_utf8(bytes).ok())
            .unwrap_or("")
    }

    /// Capacity in bytes.
    #[must_use]
    pub const fn capacity() -> usize {
        N
    }
}

impl<const N: usize> fmt::Display for InlineStr<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl<const N: usize> fmt::Debug for InlineStr<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(self.as_str(), f)
    }
}

impl<const N: usize> PartialEq<str> for InlineStr<N> {
    fn eq(&self, other: &str) -> bool {
        self.as_str() == other
    }
}

impl<const N: usize> PartialEq<&str> for InlineStr<N> {
    fn eq(&self, other: &&str) -> bool {
        self.as_str() == *other
    }
}

impl<const N: usize> From<&str> for InlineStr<N> {
    fn from(text: &str) -> Self {
        Self::new(text)
    }
}

#[cfg(feature = "serde")]
impl<const N: usize> serde::Serialize for InlineStr<N> {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_str(self.as_str())
    }
}

#[cfg(feature = "serde")]
impl<'de, const N: usize> serde::Deserialize<'de> for InlineStr<N> {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        struct Visitor<const N: usize>;

        impl<const N: usize> serde::de::Visitor<'_> for Visitor<N> {
            type Value = InlineStr<N>;

            fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "a string of at most {N} bytes")
            }

            fn visit_str<E: serde::de::Error>(self, text: &str) -> Result<Self::Value, E> {
                Ok(InlineStr::new(text))
            }
        }

        deserializer.deserialize_str(Visitor::<N>)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn an_inline_store_fills_and_then_refuses() {
        let mut store = Inline::<u8, 3>::new(0);
        assert!(store.is_empty());
        for value in 1..=3 {
            store.push(value).unwrap();
        }
        assert_eq!(store.as_slice(), &[1, 2, 3]);
        assert_eq!(store.push(4), Err(Full { capacity: 3 }));
        assert_eq!(store.as_slice(), &[1, 2, 3]);
    }

    #[test]
    fn insert_shifts_the_tail_along() {
        let mut store = Inline::<u8, 4>::new(0);
        store.push(1).unwrap();
        store.push(3).unwrap();
        store.insert(1, 2).unwrap();
        assert_eq!(store.as_slice(), &[1, 2, 3]);
        // An index past the end appends.
        store.insert(99, 4).unwrap();
        assert_eq!(store.as_slice(), &[1, 2, 3, 4]);
        assert!(store.insert(0, 5).is_err());
    }

    #[test]
    fn two_stores_are_equal_when_their_live_items_are() {
        let mut first = Inline::<u8, 8>::new(0);
        let mut second = Inline::<u8, 8>::new(9);
        first.push(1).unwrap();
        second.push(1).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn an_inline_string_truncates_on_a_character_boundary() {
        let short = InlineStr::<8>::new("north");
        assert_eq!(short.as_str(), "north");
        assert_eq!(short, "north");

        // Four bytes of capacity; each character takes two.
        let long = InlineStr::<4>::new("°°°");
        assert_eq!(long.as_str(), "°°");

        // A character that does not fit yields an empty string, never invalid
        // UTF-8.
        assert_eq!(InlineStr::<1>::new("°").as_str(), "");
    }
}
