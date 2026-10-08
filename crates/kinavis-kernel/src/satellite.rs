//! Satellites as seen by a receiver: identity, geometry, signal, use in the fix.
//!
//! NMEA `GSV`/`GSA`/`GRS`, UBX `NAV-SAT` and integrity monitoring all describe
//! the same thing: which satellites are in view, where they are in the sky,
//! how strong their signals are, which ones the receiver used, and how well
//! each range agreed with the solution. One vocabulary here lets a parser fill
//! what an integrity monitor reads, whatever the receiver's protocol.
//!
//! ```rust
//! use kinavis_kernel::satellite::{Constellation, SatelliteId, SatelliteSet, SatelliteView};
//! use kinavis_kernel::{Angle, Distance, TrueBearing};
//!
//! let mut sky: SatelliteSet = SatelliteSet::new();
//! let id = SatelliteId::new(Constellation::Galileo, 11)?;
//! let view = SatelliteView::new(id, Some(Angle::from_degrees(42.0)?), Some(TrueBearing::new(120.0)?))?
//!     .with_carrier_to_noise(44.0)?
//!     .used_in_fix(true)
//!     .with_residual(Distance::from_metres(-1.3)?);
//! sky.insert(view)?;
//!
//! assert_eq!(id.to_string(), "E11");
//! assert_eq!(sky.used().count(), 1);
//! assert_eq!(sky.get(id).and_then(|v| v.residual()).map(|r| r.metres().round()), Some(-1.0));
//! # Ok::<(), kinavis_kernel::KernelError>(())
//! ```

use core::fmt;
use core::ops::Deref;

use crate::angle::TrueBearing;
use crate::error::{ensure_range, KernelError, Result};
use crate::inline::Inline;
use crate::units::{Angle, Distance};

/// Default capacity of a [`SatelliteSet`]: four constellations of sixteen
/// satellites in view, more than a multi-band receiver tracks at once.
pub const MAX_SATELLITES: usize = 64;

/// Satellite navigation system.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Constellation {
    /// GPS (United States).
    Gps,
    /// GLONASS (Russia).
    Glonass,
    /// Galileo (European Union).
    Galileo,
    /// `BeiDou` (China).
    BeiDou,
    /// QZSS (Japan).
    Qzss,
    /// `NavIC`, formerly IRNSS (India).
    NavIc,
    /// Satellite-based augmentation: WAAS, EGNOS, MSAS, GAGAN, SDCM.
    Sbas,
}

impl Constellation {
    /// Valid satellite numbers, inclusive.
    ///
    /// GPS PRN 1–63 (IS-GPS-200), GLONASS orbital slot 1–32, Galileo 1–36,
    /// `BeiDou` PRN 1–63, QZSS 1–10 (PRN − 192), `NavIC` 1–14, SBAS PRN 120–158.
    #[must_use]
    pub const fn numbers(self) -> (u8, u8) {
        match self {
            Self::Gps | Self::BeiDou => (1, 63),
            Self::Glonass => (1, 32),
            Self::Galileo => (1, 36),
            Self::Qzss => (1, 10),
            Self::NavIc => (1, 14),
            Self::Sbas => (120, 158),
        }
    }

    /// RINEX system letter: `G`, `R`, `E`, `C`, `J`, `I`, `S`.
    #[must_use]
    pub const fn letter(self) -> char {
        match self {
            Self::Gps => 'G',
            Self::Glonass => 'R',
            Self::Galileo => 'E',
            Self::BeiDou => 'C',
            Self::Qzss => 'J',
            Self::NavIc => 'I',
            Self::Sbas => 'S',
        }
    }
}

/// Satellite: constellation and number within it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredSatelliteId", into = "StoredSatelliteId")
)]
pub struct SatelliteId {
    constellation: Constellation,
    number: u8,
}

impl SatelliteId {
    /// Satellite from constellation and number.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] outside [`Constellation::numbers`].
    pub fn new(constellation: Constellation, number: u8) -> Result<Self> {
        let (min, max) = constellation.numbers();
        ensure_range(
            "satellite number",
            f64::from(number),
            f64::from(min),
            f64::from(max),
        )?;
        Ok(Self {
            constellation,
            number,
        })
    }

    /// Constellation.
    #[must_use]
    pub const fn constellation(&self) -> Constellation {
        self.constellation
    }

    /// Number within the constellation.
    #[must_use]
    pub const fn number(&self) -> u8 {
        self.number
    }
}

impl fmt::Display for SatelliteId {
    /// Formats as in RINEX: `G05`, `R12`, `S123`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}{:02}", self.constellation.letter(), self.number)
    }
}

/// Serialised form; deserialisation goes through [`SatelliteId::new`].
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredSatelliteId {
    constellation: Constellation,
    number: u8,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredSatelliteId> for SatelliteId {
    type Error = KernelError;

    fn try_from(stored: StoredSatelliteId) -> Result<Self> {
        Self::new(stored.constellation, stored.number)
    }
}

#[cfg(feature = "serde")]
impl From<SatelliteId> for StoredSatelliteId {
    fn from(id: SatelliteId) -> Self {
        Self {
            constellation: id.constellation,
            number: id.number,
        }
    }
}

/// Highest carrier-to-noise density accepted, dB-Hz: the ceiling of the
/// receiver protocols' one-byte field.
const MAX_CARRIER_TO_NOISE: f64 = 255.0;

/// One satellite in view.
///
/// Elevation and azimuth are optional: receivers omit them for satellites
/// tracked before the almanac places them. Signal strength and residual are
/// present only when the receiver reports them.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(try_from = "StoredSatelliteView", into = "StoredSatelliteView")
)]
pub struct SatelliteView {
    id: SatelliteId,
    elevation: Option<Angle>,
    azimuth: Option<TrueBearing>,
    carrier_to_noise: Option<f64>,
    used: bool,
    residual: Option<Distance>,
}

impl SatelliteView {
    /// Fill value for unused slots of a [`SatelliteSet`].
    const PLACEHOLDER: Self = Self {
        id: SatelliteId {
            constellation: Constellation::Gps,
            number: 1,
        },
        elevation: None,
        azimuth: None,
        carrier_to_noise: None,
        used: false,
        residual: None,
    };

    /// Satellite at an elevation and azimuth, not used in the fix.
    ///
    /// # Errors
    ///
    /// [`KernelError::OutOfRange`] for an elevation outside `[-90°, 90°]`.
    pub fn new(
        id: SatelliteId,
        elevation: Option<Angle>,
        azimuth: Option<TrueBearing>,
    ) -> Result<Self> {
        if let Some(elevation) = elevation {
            ensure_range("elevation", elevation.degrees(), -90.0, 90.0)?;
        }
        Ok(Self {
            id,
            elevation,
            azimuth,
            carrier_to_noise: None,
            used: false,
            residual: None,
        })
    }

    /// Same view with a carrier-to-noise density, dB-Hz.
    ///
    /// # Errors
    ///
    /// [`KernelError::NotFinite`] or [`KernelError::OutOfRange`] outside
    /// `[0, 255]`.
    pub fn with_carrier_to_noise(mut self, db_hz: f64) -> Result<Self> {
        ensure_range("carrier-to-noise density", db_hz, 0.0, MAX_CARRIER_TO_NOISE)?;
        self.carrier_to_noise = Some(db_hz);
        Ok(self)
    }

    /// Same view, marked as used or not used in the fix.
    #[must_use]
    pub const fn used_in_fix(mut self, used: bool) -> Self {
        self.used = used;
        self
    }

    /// Same view with a range residual: measured minus computed range after
    /// the fix.
    #[must_use]
    pub const fn with_residual(mut self, residual: Distance) -> Self {
        self.residual = Some(residual);
        self
    }

    /// Satellite.
    #[must_use]
    pub const fn id(&self) -> SatelliteId {
        self.id
    }

    /// Elevation above the horizon, in `[-90°, 90°]`.
    #[must_use]
    pub const fn elevation(&self) -> Option<Angle> {
        self.elevation
    }

    /// True azimuth.
    #[must_use]
    pub const fn azimuth(&self) -> Option<TrueBearing> {
        self.azimuth
    }

    /// Carrier-to-noise density, dB-Hz.
    #[must_use]
    pub const fn carrier_to_noise(&self) -> Option<f64> {
        self.carrier_to_noise
    }

    /// Whether the receiver used this satellite in the fix.
    #[must_use]
    pub const fn is_used(&self) -> bool {
        self.used
    }

    /// Range residual, if reported.
    #[must_use]
    pub const fn residual(&self) -> Option<Distance> {
        self.residual
    }
}

/// Serialised form; deserialisation goes through [`SatelliteView::new`] and
/// [`SatelliteView::with_carrier_to_noise`].
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct StoredSatelliteView {
    id: SatelliteId,
    elevation: Option<Angle>,
    azimuth: Option<TrueBearing>,
    carrier_to_noise: Option<f64>,
    used: bool,
    residual: Option<Distance>,
}

#[cfg(feature = "serde")]
impl TryFrom<StoredSatelliteView> for SatelliteView {
    type Error = KernelError;

    fn try_from(stored: StoredSatelliteView) -> Result<Self> {
        let mut view = Self::new(stored.id, stored.elevation, stored.azimuth)?;
        if let Some(db_hz) = stored.carrier_to_noise {
            view = view.with_carrier_to_noise(db_hz)?;
        }
        view = view.used_in_fix(stored.used);
        if let Some(residual) = stored.residual {
            view = view.with_residual(residual);
        }
        Ok(view)
    }
}

#[cfg(feature = "serde")]
impl From<SatelliteView> for StoredSatelliteView {
    fn from(view: SatelliteView) -> Self {
        Self {
            id: view.id,
            elevation: view.elevation,
            azimuth: view.azimuth,
            carrier_to_noise: view.carrier_to_noise,
            used: view.used,
            residual: view.residual,
        }
    }
}

/// Satellites in view, at most one view per satellite, fixed capacity `N`.
///
/// Dereferences to a slice in insertion order. Not `Copy`: at about 4.5 KiB
/// it is passed by reference.
#[derive(Clone)]
pub struct SatelliteSet<const N: usize = MAX_SATELLITES> {
    views: Inline<SatelliteView, N>,
}

impl<const N: usize> SatelliteSet<N> {
    /// Empty set.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            views: Inline::new(SatelliteView::PLACEHOLDER),
        }
    }

    /// Adds a view, replacing an earlier view of the same satellite in place.
    ///
    /// # Errors
    ///
    /// [`KernelError::CapacityExceeded`] when the satellite is new and `N`
    /// views are held; the set is unchanged.
    pub fn insert(&mut self, view: SatelliteView) -> Result<()> {
        if let Some(slot) = self
            .views
            .as_mut_slice()
            .iter_mut()
            .find(|held| held.id == view.id)
        {
            *slot = view;
            return Ok(());
        }
        self.views
            .push(view)
            .map_err(|full| KernelError::CapacityExceeded {
                context: "satellite set",
                needed: full.capacity.saturating_add(1),
                capacity: full.capacity,
            })
    }

    /// View of a satellite, if present.
    #[must_use]
    pub fn get(&self, id: SatelliteId) -> Option<&SatelliteView> {
        self.as_slice().iter().find(|view| view.id == id)
    }

    /// Views of the satellites used in the fix.
    pub fn used(&self) -> impl Iterator<Item = &SatelliteView> {
        self.as_slice().iter().filter(|view| view.used)
    }

    /// Views, in insertion order.
    #[must_use]
    pub fn as_slice(&self) -> &[SatelliteView] {
        self.views.as_slice()
    }
}

impl<const N: usize> Default for SatelliteSet<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Deref for SatelliteSet<N> {
    type Target = [SatelliteView];

    fn deref(&self) -> &[SatelliteView] {
        self.as_slice()
    }
}

impl<'a, const N: usize> IntoIterator for &'a SatelliteSet<N> {
    type Item = &'a SatelliteView;
    type IntoIter = core::slice::Iter<'a, SatelliteView>;

    fn into_iter(self) -> Self::IntoIter {
        self.as_slice().iter()
    }
}

impl<const N: usize> fmt::Debug for SatelliteSet<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_list().entries(self.as_slice()).finish()
    }
}

impl<const N: usize, const M: usize> PartialEq<SatelliteSet<M>> for SatelliteSet<N> {
    /// Equal when they hold the same views in the same order, regardless of
    /// capacity.
    fn eq(&self, other: &SatelliteSet<M>) -> bool {
        self.as_slice() == other.as_slice()
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use alloc::format;

    fn id(constellation: Constellation, number: u8) -> SatelliteId {
        SatelliteId::new(constellation, number).unwrap()
    }

    #[test]
    fn numbers_are_checked_per_constellation() {
        assert!(SatelliteId::new(Constellation::Gps, 0).is_err());
        assert!(SatelliteId::new(Constellation::Gps, 32).is_ok());
        assert!(SatelliteId::new(Constellation::Galileo, 37).is_err());
        assert!(SatelliteId::new(Constellation::Qzss, 193).is_err());
        assert!(SatelliteId::new(Constellation::Sbas, 119).is_err());
        assert_eq!(format!("{}", id(Constellation::Sbas, 123)), "S123");
        assert_eq!(format!("{}", id(Constellation::Glonass, 7)), "R07");
    }

    #[test]
    fn a_view_rejects_an_elevation_beyond_the_zenith_and_a_bad_signal() {
        let gps = id(Constellation::Gps, 5);
        assert!(SatelliteView::new(gps, Some(Angle::from_degrees(90.5).unwrap()), None).is_err());
        let view = SatelliteView::new(gps, Some(Angle::from_degrees(-2.0).unwrap()), None).unwrap();
        assert!(view.with_carrier_to_noise(-1.0).is_err());
        assert!(view.with_carrier_to_noise(f64::NAN).is_err());
        assert_eq!(
            view.with_carrier_to_noise(38.0).unwrap().carrier_to_noise(),
            Some(38.0)
        );
        assert!(!view.is_used());
    }

    #[test]
    fn a_satellite_seen_again_replaces_its_earlier_view() {
        let mut set: SatelliteSet<2> = SatelliteSet::new();
        let first = SatelliteView::new(id(Constellation::Gps, 5), None, None).unwrap();
        let second = SatelliteView::new(id(Constellation::BeiDou, 5), None, None).unwrap();
        set.insert(first).unwrap();
        set.insert(second).unwrap();
        set.insert(first.used_in_fix(true)).unwrap();
        assert_eq!(set.len(), 2);
        assert!(set.get(first.id()).unwrap().is_used());
        assert_eq!(set.used().count(), 1);
        let third = SatelliteView::new(id(Constellation::Galileo, 1), None, None).unwrap();
        assert!(matches!(
            set.insert(third),
            Err(KernelError::CapacityExceeded { capacity: 2, .. })
        ));
        assert_eq!(set.len(), 2);
        assert!(format!("{set:?}").starts_with('['));
        let other: SatelliteSet = SatelliteSet::new();
        assert_ne!(set, other);
    }
}
