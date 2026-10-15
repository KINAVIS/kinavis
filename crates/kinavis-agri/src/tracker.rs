//! Following passes: the pass being worked, cross-track and heading error,
//! distance to the end, events.

use kinavis::region::PlanarPoint;
use kinavis_kernel::error::ensure_range;
use kinavis_kernel::event::{Event, EventList};
use kinavis_kernel::{math, Angle, Distance, GridCourse, Instant, Utc};

use crate::error::Result;
use crate::field::Field;
use crate::line::{metres, ReferenceLine};
use crate::pass::{PassIndex, PassPattern};

/// Events of field guidance.
///
/// `#[non_exhaustive]`; match with a wildcard arm.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AgriEvent {
    /// The tracker moved to another pass.
    PassChanged {
        /// Pass left.
        from: PassIndex,
        /// Pass taken up.
        to: PassIndex,
        /// Time of the position.
        at: Instant<Utc>,
    },
    /// The end of the pass is within the warning distance.
    EndOfPassApproaching {
        /// Distance to the boundary ahead.
        remaining: Distance,
        /// Time of the position.
        at: Instant<Utc>,
    },
    /// Cross-track error went beyond the limit.
    OffLine {
        /// Cross-track error, positive right of the pass.
        cross_track: Distance,
        /// Limit exceeded.
        limit: Distance,
        /// Time of the position.
        at: Instant<Utc>,
    },
}

impl Event for AgriEvent {
    const PLACEHOLDER: Self = Self::PassChanged {
        from: PassIndex::REFERENCE,
        to: PassIndex::REFERENCE,
        at: Instant::UNIX_EPOCH,
    };

    fn at(&self) -> Instant<Utc> {
        match self {
            Self::PassChanged { at, .. }
            | Self::EndOfPassApproaching { at, .. }
            | Self::OffLine { at, .. } => *at,
        }
    }
}

/// Most events one update reports.
pub const MAX_AGRI_EVENTS: usize = 3;

/// Where the machine is on its pass.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PassView {
    /// Pass being worked.
    pub pass: PassIndex,
    /// Distance from the pass, positive right of it looking along the travel.
    pub cross_track: Distance,
    /// Heading less the pass direction, wrapped to ±180°, positive when
    /// turned right of the pass.
    pub heading_error: Angle,
    /// Direction of the pass in the direction of travel.
    pub course: GridCourse,
    /// Whether the travel is against the line's reference direction.
    pub reversed: bool,
    /// Distance to the field boundary or an obstacle along the pass
    /// direction, when a field is given and the pass has an end.
    pub to_end: Option<Distance>,
}

/// Pass of a line nearest to a point, and the machine's place on it.
///
/// A pure function of the inputs; [`PassTracker`] adds hysteresis and events.
/// The direction of travel is taken from the heading: within 90° of the
/// line's reference direction is forward.
#[must_use]
pub fn locate<L: ReferenceLine>(
    line: &L,
    pattern: &PassPattern,
    point: PlanarPoint,
    heading: GridCourse,
) -> PassView {
    let station = line.station(point);
    let pass = pattern.pass_at_metres(station.offset.metres());
    locate_pass(line, pattern, point, heading, pass)
}

/// Thresholds of a [`PassTracker`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackerConfig {
    off_line_limit: f64,
    end_warning: f64,
    hysteresis: f64,
}

impl TrackerConfig {
    /// Thresholds.
    ///
    /// - `off_line_limit`: cross-track error reported by
    ///   [`AgriEvent::OffLine`];
    /// - `end_warning`: distance to the end reported by
    ///   [`AgriEvent::EndOfPassApproaching`];
    /// - `hysteresis`: how far past the midpoint between passes the machine
    ///   must go before the tracker moves to the next pass, and how far back
    ///   inside a threshold before its event can be reported again.
    ///
    /// # Errors
    ///
    /// [`kinavis_kernel::KernelError::OutOfRange`] for a limit or warning not
    /// above zero, or a negative hysteresis.
    pub fn new(
        off_line_limit: Distance,
        end_warning: Distance,
        hysteresis: Distance,
    ) -> Result<Self> {
        ensure_range("off-line limit, metres", off_line_limit.metres(), 1e-3, 1e4)?;
        ensure_range(
            "end-of-pass warning, metres",
            end_warning.metres(),
            1e-3,
            1e5,
        )?;
        ensure_range("hysteresis, metres", hysteresis.metres(), 0.0, 1e3)?;
        Ok(Self {
            off_line_limit: off_line_limit.metres(),
            end_warning: end_warning.metres(),
            hysteresis: hysteresis.metres(),
        })
    }

    /// Cross-track limit.
    #[must_use]
    pub fn off_line_limit(&self) -> Distance {
        metres(self.off_line_limit)
    }

    /// End-of-pass warning distance.
    #[must_use]
    pub fn end_warning(&self) -> Distance {
        metres(self.end_warning)
    }

    /// Hysteresis.
    #[must_use]
    pub fn hysteresis(&self) -> Distance {
        metres(self.hysteresis)
    }
}

/// Pass being worked, kept between positions.
///
/// Moves to the neighbouring pass only once the machine is past the midpoint
/// by the hysteresis, so noise at the midpoint does not flip it, and not at
/// all while locked. Events are reported once per crossing of a threshold.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PassTracker {
    config: TrackerConfig,
    current: Option<PassIndex>,
    locked: bool,
    off_line: bool,
    end_reported: bool,
}

impl PassTracker {
    /// Tracker without a pass; the first update takes the nearest.
    #[must_use]
    pub const fn new(config: TrackerConfig) -> Self {
        Self {
            config,
            current: None,
            locked: false,
            off_line: false,
            end_reported: false,
        }
    }

    /// Pass being worked, if any update has been made.
    #[must_use]
    pub const fn current(&self) -> Option<PassIndex> {
        self.current
    }

    /// Holds a pass until [`PassTracker::unlock`]: guidance stays on it
    /// whatever the offset.
    pub fn lock(&mut self, pass: PassIndex) {
        self.take(pass);
        self.locked = true;
    }

    /// Lets the tracker move between passes again.
    pub fn unlock(&mut self) {
        self.locked = false;
    }

    /// Whether a pass is locked.
    #[must_use]
    pub const fn is_locked(&self) -> bool {
        self.locked
    }

    /// Updates with a position of the control point, without a field: no
    /// distance to the end.
    pub fn update<L: ReferenceLine>(
        &mut self,
        line: &L,
        pattern: &PassPattern,
        point: PlanarPoint,
        heading: GridCourse,
        at: Instant<Utc>,
    ) -> (PassView, EventList<AgriEvent, MAX_AGRI_EVENTS>) {
        self.step(line, pattern, point, heading, at, |_, _| None)
    }

    /// Updates with a position of the control point inside a field.
    ///
    /// # Errors
    ///
    /// [`crate::AgriError::PlaneMismatch`] if the line and the field are in
    /// different planes.
    pub fn update_within<L: ReferenceLine, const N: usize, const H: usize, const K: usize>(
        &mut self,
        line: &L,
        pattern: &PassPattern,
        field: &Field<N, H, K>,
        point: PlanarPoint,
        heading: GridCourse,
        at: Instant<Utc>,
    ) -> Result<(PassView, EventList<AgriEvent, MAX_AGRI_EVENTS>)> {
        line.plane().ensure_same(field.plane())?;
        let closed = line.is_closed();
        Ok(
            self.step(line, pattern, point, heading, at, |from, course| {
                if closed {
                    None
                } else {
                    field.distance_to_exit(from, course)
                }
            }),
        )
    }

    fn step<L: ReferenceLine>(
        &mut self,
        line: &L,
        pattern: &PassPattern,
        point: PlanarPoint,
        heading: GridCourse,
        at: Instant<Utc>,
        to_end: impl Fn(PlanarPoint, GridCourse) -> Option<Distance>,
    ) -> (PassView, EventList<AgriEvent, MAX_AGRI_EVENTS>) {
        let mut events = EventList::with_capacity();
        let offset = line.station(point).offset.metres();
        let nearest = pattern.pass_at_metres(offset);
        let pass = match self.current {
            None => {
                self.take(nearest);
                nearest
            }
            Some(current) if self.locked || current == nearest => current,
            Some(current) => {
                let reach = 0.5 * pattern.spacing_metres() + self.config.hysteresis;
                if math::abs(offset - pattern.offset_metres(current)) > reach {
                    events.push(AgriEvent::PassChanged {
                        from: current,
                        to: nearest,
                        at,
                    });
                    self.take(nearest);
                    nearest
                } else {
                    current
                }
            }
        };
        let mut view = locate_pass(line, pattern, point, heading, pass);
        view.to_end = to_end(point, view.course);

        let error = math::abs(view.cross_track.metres());
        if !self.off_line && error > self.config.off_line_limit {
            self.off_line = true;
            events.push(AgriEvent::OffLine {
                cross_track: view.cross_track,
                limit: self.config.off_line_limit(),
                at,
            });
        } else if self.off_line && error <= self.config.off_line_limit - self.config.hysteresis {
            self.off_line = false;
        }
        match view.to_end.map(Distance::metres) {
            Some(remaining) if !self.end_reported && remaining <= self.config.end_warning => {
                self.end_reported = true;
                events.push(AgriEvent::EndOfPassApproaching {
                    remaining: metres(remaining),
                    at,
                });
            }
            Some(remaining) if remaining > self.config.end_warning + self.config.hysteresis => {
                self.end_reported = false;
            }
            _ => {}
        }
        (view, events)
    }

    fn take(&mut self, pass: PassIndex) {
        if self.current != Some(pass) {
            self.current = Some(pass);
            self.off_line = false;
            self.end_reported = false;
        }
    }
}

/// Place on a given pass.
pub(crate) fn locate_pass<L: ReferenceLine>(
    line: &L,
    pattern: &PassPattern,
    point: PlanarPoint,
    heading: GridCourse,
    pass: PassIndex,
) -> PassView {
    let station = line.station(point);
    let left_of_pass = station.offset.metres() - pattern.offset_metres(pass);
    let reversed = station.direction.angular_distance(heading) > 90.0;
    let (course, right_of_travel) = if reversed {
        (station.direction.reciprocal(), left_of_pass)
    } else {
        (station.direction, -left_of_pass)
    };
    PassView {
        pass,
        cross_track: metres(right_of_travel),
        heading_error: Angle::from_degrees(course.signed_difference(heading))
            .unwrap_or(Angle::ZERO),
        course,
        reversed,
        to_end: None,
    }
}
