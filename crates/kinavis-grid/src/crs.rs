//! A datum and a transverse Mercator projection on its ellipsoid: national
//! grids, with Gauss–Krüger zones and the common presets.

use kinavis_kernel::error::{ensure_range, Result};
use kinavis_kernel::geodesy::Datum;
use kinavis_kernel::{Distance, Latitude, Longitude, Position};

use crate::tm::{Projected, TmParameters, TransverseMercator};

/// Projected coordinate reference system: datum plus transverse Mercator.
///
/// [`ProjectedCrs::project`] takes a WGS 84 position (a GNSS fix), shifts it
/// to the datum and projects it; [`ProjectedCrs::unproject`] reverses both.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProjectedCrs {
    datum: Datum,
    projection: TransverseMercator,
}

impl ProjectedCrs {
    /// Grid from a datum and projection parameters.
    ///
    /// # Errors
    ///
    /// As [`TransverseMercator::new`].
    pub fn new(datum: Datum, parameters: TmParameters) -> Result<Self> {
        Ok(Self {
            projection: TransverseMercator::new(datum.ellipsoid(), parameters)?,
            datum,
        })
    }

    /// Gauss–Krüger zone of 6°: central meridian `6n − 3`, scale 1, false
    /// easting `n · 10⁶ + 500 000` m (zone number in front).
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] for a zone outside 1–60.
    pub fn gauss_kruger_6(datum: Datum, zone: u8) -> Result<Self> {
        ensure_range("Gauss-Kruger 6-degree zone", f64::from(zone), 1.0, 60.0)?;
        Self::gauss_kruger(datum, zone, 6.0 * f64::from(zone) - 3.0)
    }

    /// Gauss–Krüger zone of 3°: central meridian `3n`, scale 1, false easting
    /// `n · 10⁶ + 500 000` m.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] for a zone outside 1–120.
    pub fn gauss_kruger_3(datum: Datum, zone: u8) -> Result<Self> {
        ensure_range("Gauss-Kruger 3-degree zone", f64::from(zone), 1.0, 120.0)?;
        Self::gauss_kruger(datum, zone, 3.0 * f64::from(zone))
    }

    fn gauss_kruger(datum: Datum, zone: u8, central_meridian: f64) -> Result<Self> {
        Self::new(
            datum,
            TmParameters {
                central_meridian: Longitude::from_degrees(central_meridian)?,
                latitude_of_origin: Latitude::EQUATOR,
                scale: 1.0,
                false_easting: Distance::from_metres(f64::from(zone) * 1e6 + 500_000.0)?,
                false_northing: Distance::ZERO,
            },
        )
    }

    /// СК-42 (Pulkovo 1942), 6° Gauss–Krüger zone: EPSG 28402–28432 for
    /// zones 2–32.
    ///
    /// # Errors
    ///
    /// As [`ProjectedCrs::gauss_kruger_6`].
    pub fn pulkovo_1942(zone: u8) -> Result<Self> {
        Self::gauss_kruger_6(Datum::PULKOVO_1942, zone)
    }

    /// ГСК-2011, 6° Gauss–Krüger zone: EPSG 20904–20932 for zones 4–32.
    ///
    /// # Errors
    ///
    /// As [`ProjectedCrs::gauss_kruger_6`].
    pub fn gsk_2011(zone: u8) -> Result<Self> {
        Self::gauss_kruger_6(Datum::GSK_2011, zone)
    }

    /// СК-95 (Pulkovo 1995), 6° Gauss–Krüger zone: EPSG 20004–20032 for
    /// zones 4–32.
    ///
    /// # Errors
    ///
    /// As [`ProjectedCrs::gauss_kruger_6`].
    pub fn pulkovo_1995(zone: u8) -> Result<Self> {
        Self::gauss_kruger_6(Datum::PULKOVO_1995, zone)
    }

    /// DHDN, 3° Gauss–Krüger zone (Germany): EPSG 31466–31469 for zones 2–5.
    ///
    /// # Errors
    ///
    /// As [`ProjectedCrs::gauss_kruger_3`].
    pub fn dhdn(zone: u8) -> Result<Self> {
        Self::gauss_kruger_3(Datum::DHDN, zone)
    }

    /// British National Grid on OSGB36: EPSG 27700.
    ///
    /// # Errors
    ///
    /// Not reachable: the parameters are constants.
    pub fn british_national_grid() -> Result<Self> {
        Self::new(
            Datum::OSGB36,
            TmParameters {
                central_meridian: Longitude::from_degrees(-2.0)?,
                latitude_of_origin: Latitude::from_degrees(49.0)?,
                scale: 0.999_601_271_7,
                false_easting: Distance::from_metres(400_000.0)?,
                false_northing: Distance::from_metres(-100_000.0)?,
            },
        )
    }

    /// Datum.
    #[must_use]
    pub const fn datum(&self) -> &Datum {
        &self.datum
    }

    /// Projection on the datum's ellipsoid.
    #[must_use]
    pub const fn projection(&self) -> &TransverseMercator {
        &self.projection
    }

    /// Grid coordinates of a WGS 84 position: datum shift, then projection.
    ///
    /// # Errors
    ///
    /// As [`Datum::from_wgs84`] and [`TransverseMercator::project`].
    pub fn project(&self, wgs84: Position) -> Result<Projected> {
        let on_datum = self.datum.from_wgs84(wgs84)?;
        self.projection
            .project(on_datum.latitude(), on_datum.longitude())
    }

    /// WGS 84 position of grid coordinates.
    ///
    /// # Errors
    ///
    /// As [`TransverseMercator::unproject`] and [`Datum::to_wgs84`].
    pub fn unproject(&self, easting: Distance, northing: Distance) -> Result<Position> {
        let (latitude, longitude, _, _) = self.projection.unproject(easting, northing)?;
        self.datum.to_wgs84(Position::new(latitude, longitude))
    }
}
