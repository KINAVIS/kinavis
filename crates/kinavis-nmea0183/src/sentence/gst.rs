//! `GST` — pseudorange error statistics.
//!
//! ```text
//! $GPGST,hhmmss.ss,rms,major,minor,orientation,σlat,σlon,σalt*hh
//! ```
//!
//! The receiver's own estimate of its position error: the 1σ error ellipse
//! and the standard deviations of latitude, longitude and altitude, in metres.
//! Converted to the kernel's [`ErrorEllipse`], it replaces the nominal
//! HDOP × UERE.

use core::fmt::{self, Write};

use kinavis_kernel::snapshot::ErrorEllipse;
use kinavis_kernel::{Distance, TrueCourse};

use crate::encode;
use crate::error::NmeaError;
use crate::field::{bounds, Field};
use crate::frame::Frame;
use crate::sentence::{Talker, TimeOfDay};

/// Pseudorange error statistics.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gst {
    /// Talker.
    pub talker: Talker,
    /// Time of the fix, UTC.
    pub time: Option<TimeOfDay>,
    /// RMS of the pseudorange residuals.
    pub range_rms: Option<Distance>,
    /// 1σ semi-major axis of the error ellipse.
    pub semi_major: Option<Distance>,
    /// 1σ semi-minor axis of the error ellipse.
    pub semi_minor: Option<Distance>,
    /// True direction of the semi-major axis.
    pub orientation: Option<TrueCourse>,
    /// 1σ latitude error.
    pub latitude_sigma: Option<Distance>,
    /// 1σ longitude error.
    pub longitude_sigma: Option<Distance>,
    /// 1σ altitude error.
    pub altitude_sigma: Option<Distance>,
}

impl Gst {
    /// Minimum field count: through the longitude error.
    const REQUIRED: usize = 7;

    pub(crate) fn decode(talker: Talker, frame: Frame<'_>) -> Result<Self, NmeaError> {
        let found = frame.fields().remaining();
        if found < Self::REQUIRED {
            return Err(NmeaError::TooFewFields {
                found,
                required: Self::REQUIRED,
            });
        }
        let mut fields = frame.fields().indexed();
        let mut next = || {
            fields.next().unwrap_or(Field {
                bytes: &[],
                index: usize::MAX,
            })
        };
        Ok(Self {
            talker,
            time: next().optional_time_of_day()?,
            range_rms: sigma(next(), "range RMS")?,
            semi_major: sigma(next(), "semi-major axis")?,
            semi_minor: sigma(next(), "semi-minor axis")?,
            orientation: orientation(next())?,
            latitude_sigma: sigma(next(), "latitude error")?,
            longitude_sigma: sigma(next(), "longitude error")?,
            altitude_sigma: sigma(next(), "altitude error")?,
        })
    }

    /// The horizontal error ellipse, if the sentence carries one.
    ///
    /// From the semi-axes and orientation when present (axes swapped and the
    /// orientation turned 90° if the receiver wrote the minor axis larger);
    /// otherwise from the latitude and longitude errors, uncorrelated.
    #[must_use]
    pub fn error_ellipse(&self) -> Option<ErrorEllipse> {
        if let (Some(major), Some(minor)) = (self.semi_major, self.semi_minor) {
            let orientation = self.orientation.unwrap_or(TrueCourse::NORTH);
            return if major.metres() >= minor.metres() {
                ErrorEllipse::new(major, minor, orientation).ok()
            } else {
                ErrorEllipse::new(minor, major, orientation.offset(90.0).ok()?).ok()
            };
        }
        let latitude = self.latitude_sigma?.metres();
        let longitude = self.longitude_sigma?.metres();
        ErrorEllipse::from_covariance(latitude * latitude, 0.0, longitude * longitude)
    }
}

/// A standard deviation or RMS; above [`bounds::SIGMA_METRES`] it reads as
/// not available: receivers write hundreds of kilometres before a fix
/// (u-blox LEA-5Q) or in the RMS beside a valid ellipse (u-blox M9).
fn sigma(field: Field<'_>, expected: &'static str) -> Result<Option<Distance>, NmeaError> {
    if field.is_empty() {
        return Ok(None);
    }
    if field.decimal(expected)? > bounds::SIGMA_METRES.1 {
        return Ok(None);
    }
    field.optional_metres(expected, bounds::SIGMA_METRES)
}

/// Direction of the semi-major axis, read modulo 360°: an axis has no sense,
/// and SkyTraq writes it signed.
fn orientation(field: Field<'_>) -> Result<Option<TrueCourse>, NmeaError> {
    if field.is_empty() {
        return Ok(None);
    }
    let degrees = field.bounded_decimal("orientation", (-360.0, 360.0))?;
    TrueCourse::wrap(degrees)
        .map(Some)
        .map_err(|error| NmeaError::Value {
            index: field.index,
            error,
        })
}

impl fmt::Display for Gst {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let metres = |value: Option<Distance>| value.map(|d| ThreeDecimals(d.metres()));
        encode::sentence(f, |out| {
            write!(out, "{}GST,", self.talker)?;
            encode::optional(out, self.time)?;
            for value in [self.range_rms, self.semi_major, self.semi_minor] {
                out.write_char(',')?;
                encode::optional(out, metres(value))?;
            }
            out.write_char(',')?;
            encode::optional(out, self.orientation.map(|c| Orientation(c.degrees())))?;
            for value in [
                self.latitude_sigma,
                self.longitude_sigma,
                self.altitude_sigma,
            ] {
                out.write_char(',')?;
                encode::optional(out, metres(value))?;
            }
            Ok(())
        })
    }
}

struct ThreeDecimals(f64);

impl fmt::Display for ThreeDecimals {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.3}", self.0)
    }
}

/// Orientation to a tenth of a degree, `360.0` written as `0.0`.
struct Orientation(f64);

impl fmt::Display for Orientation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let tenths = kinavis_kernel::math::round(self.0 * 10.0) / 10.0;
        write!(f, "{:.1}", if tenths >= 360.0 { 0.0 } else { tenths })
    }
}
