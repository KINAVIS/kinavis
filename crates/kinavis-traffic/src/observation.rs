//! Single target observation: position and time.

use kinavis::error::Result;
use kinavis::relative_motion::Contact;
use kinavis::sailings::rhumb_destination;
use kinavis_kernel::angle::TrueCourse;
use kinavis_kernel::event::TargetId;
use kinavis_kernel::position::Position;
use kinavis_kernel::snapshot::GroundTrack;
use kinavis_kernel::time::{Instant, Utc};

/// One sighting of a target: radar bearing and range, AIS position report, or
/// any other position source.
///
/// Position is the common denominator from which tracks are built. Sources that
/// also report course and speed over ground (AIS) add them; the track prefers
/// them to values fitted from positions.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TargetObservation {
    target: TargetId,
    position: Position,
    at: Instant<Utc>,
    ground_track: Option<GroundTrack>,
    heading: Option<TrueCourse>,
}

impl TargetObservation {
    /// Target at a position at an instant.
    #[must_use]
    pub const fn new(target: TargetId, position: Position, at: Instant<Utc>) -> Self {
        Self {
            target,
            position,
            at,
            ground_track: None,
            heading: None,
        }
    }

    /// Radar plot: true bearing and range from own position at the time of the
    /// plot.
    ///
    /// # Errors
    ///
    /// As [`rhumb_destination`]: a plot beyond a pole.
    pub fn from_contact(
        target: TargetId,
        own: Position,
        contact: Contact,
        at: Instant<Utc>,
    ) -> Result<Self> {
        let position = rhumb_destination(
            own,
            TrueCourse::new(contact.bearing.degrees())?,
            contact.range,
        )?;
        Ok(Self::new(target, position, at))
    }

    /// Adds the target's reported course and speed over ground.
    #[must_use]
    pub const fn with_ground_track(mut self, ground_track: GroundTrack) -> Self {
        self.ground_track = Some(ground_track);
        self
    }

    /// Adds the target's reported heading.
    #[must_use]
    pub const fn with_heading(mut self, heading: TrueCourse) -> Self {
        self.heading = Some(heading);
        self
    }

    /// Target.
    #[must_use]
    pub const fn target(&self) -> TargetId {
        self.target
    }

    /// Position.
    #[must_use]
    pub const fn position(&self) -> Position {
        self.position
    }

    /// Time.
    #[must_use]
    pub const fn at(&self) -> Instant<Utc> {
        self.at
    }

    /// Reported course and speed, if any.
    #[must_use]
    pub const fn ground_track(&self) -> Option<GroundTrack> {
        self.ground_track
    }

    /// Reported heading, if any.
    #[must_use]
    pub const fn heading(&self) -> Option<TrueCourse> {
        self.heading
    }
}
