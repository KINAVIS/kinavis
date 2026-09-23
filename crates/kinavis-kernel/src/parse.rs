//! Parsing angles and positions in written notation.
//!
//! One parser serves latitude, longitude and plain angles: whole degrees,
//! optional minutes, optional seconds, optional hemisphere letter at either
//! end.
//!
//! The NMEA run-together form (`5045.300` for 50°45.3′) is not accepted: it is
//! indistinguishable from decimal degrees.

use crate::error::{Excerpt, KernelError, Result};

/// Maximum input length. Real positions are far shorter; the limit bounds work
/// on pathological input.
const MAX_INPUT: usize = 64;

/// Parsed sexagesimal value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Sexagesimal {
    /// Magnitude in degrees.
    pub(crate) magnitude: f64,
    /// Hemisphere letter, upper case, if present.
    pub(crate) hemisphere: Option<char>,
    /// Explicit minus sign.
    pub(crate) negative: bool,
}

impl Sexagesimal {
    /// Signed value from the hemisphere letter or sign.
    ///
    /// `negative_letters`: letters meaning negative, `"S"` for latitude, `"W"`
    /// for longitude.
    pub(crate) fn signed(self, negative_letters: &str) -> f64 {
        let negative = self.negative
            || self
                .hemisphere
                .is_some_and(|letter| negative_letters.contains(letter));
        if negative {
            -self.magnitude
        } else {
            self.magnitude
        }
    }
}

/// Parses `50°45.3'`, `50 45 18`, `-50.755`, `N50 45.3` etc.
///
/// # Errors
///
/// [`KernelError::Parse`] for unreadable input.
pub(crate) fn sexagesimal(what: &'static str, input: &str) -> Result<Sexagesimal> {
    let trimmed = input.trim();
    if trimmed.is_empty() || trimmed.len() > MAX_INPUT {
        return Err(parse_error(what, input));
    }

    let mut hemisphere = None;
    let mut body = trimmed;

    // One hemisphere letter, leading or trailing.
    if let Some(letter) = leading_hemisphere(body) {
        hemisphere = Some(letter);
        body = body.get(1..).unwrap_or_default().trim_start();
    }
    if let Some(letter) = trailing_hemisphere(body) {
        if hemisphere.is_some() {
            return Err(parse_error(what, input));
        }
        hemisphere = Some(letter);
        body = body
            .get(..body.len().saturating_sub(1))
            .unwrap_or_default()
            .trim_end();
    }
    if body.chars().any(is_hemisphere_letter) {
        return Err(parse_error(what, input));
    }

    let mut negative = false;
    if let Some(rest) = body.strip_prefix('-') {
        negative = true;
        body = rest.trim_start();
    } else if let Some(rest) = body.strip_prefix('+') {
        body = rest.trim_start();
    }
    // Sign and hemisphere together are redundant and may conflict.
    if negative && hemisphere.is_some() {
        return Err(parse_error(what, input));
    }

    let groups = numeric_groups(body).ok_or_else(|| parse_error(what, input))?;
    let magnitude = combine(&groups).ok_or_else(|| parse_error(what, input))?;

    Ok(Sexagesimal {
        magnitude,
        hemisphere,
        negative,
    })
}

/// Splits a position into latitude and longitude parts.
///
/// # Errors
///
/// [`KernelError::Parse`] if the halves cannot be separated.
pub(crate) fn split_position(input: &str) -> Result<(&str, &str)> {
    let trimmed = input.trim();
    if trimmed.is_empty() || trimmed.len() > MAX_INPUT * 2 {
        return Err(parse_error("position", input));
    }

    let north_south = trimmed
        .char_indices()
        .find(|(_, c)| matches!(c, 'N' | 'n' | 'S' | 's'));
    let east_west = trimmed
        .char_indices()
        .find(|(_, c)| matches!(c, 'E' | 'e' | 'W' | 'w'));

    match (north_south, east_west) {
        (Some((latitude_index, letter)), Some((longitude_index, _))) => {
            if longitude_index <= latitude_index {
                // Longitude first, or interleaved letters: not supported.
                return Err(parse_error("position", input));
            }
            let split = if latitude_index == 0 {
                // Leading hemisphere: `N50 45.3 W001 17.8`.
                longitude_index
            } else {
                // Trailing hemisphere: `50 45.3 N 001 17.8 W`.
                latitude_index.saturating_add(letter.len_utf8())
            };
            let latitude = trimmed
                .get(..split)
                .ok_or_else(|| parse_error("position", input))?;
            let longitude = trimmed
                .get(split..)
                .ok_or_else(|| parse_error("position", input))?;
            Ok((latitude.trim(), longitude.trim()))
        }
        (None, None) => {
            // Decimal degrees separated by a comma or space.
            let mut parts = trimmed.split([',', ';']).map(str::trim);
            let (Some(latitude), Some(longitude), None) =
                (parts.next(), parts.next(), parts.next())
            else {
                let mut words = trimmed.split_whitespace();
                let (Some(latitude), Some(longitude), None) =
                    (words.next(), words.next(), words.next())
                else {
                    return Err(parse_error("position", input));
                };
                return Ok((latitude, longitude));
            };
            Ok((latitude, longitude))
        }
        _ => Err(parse_error("position", input)),
    }
}

/// Builds the error with a bounded excerpt of the input.
pub(crate) fn parse_error(what: &'static str, input: &str) -> KernelError {
    KernelError::Parse {
        what,
        input: Excerpt::new(input),
    }
}

fn is_hemisphere_letter(character: char) -> bool {
    matches!(character, 'N' | 'n' | 'S' | 's' | 'E' | 'e' | 'W' | 'w')
}

fn leading_hemisphere(body: &str) -> Option<char> {
    let first = body.chars().next()?;
    is_hemisphere_letter(first).then(|| first.to_ascii_uppercase())
}

fn trailing_hemisphere(body: &str) -> Option<char> {
    let last = body.chars().next_back()?;
    is_hemisphere_letter(last).then(|| last.to_ascii_uppercase())
}

/// Splits the body into one to three numeric groups, as slices of `body` (no
/// allocation).
fn numeric_groups(body: &str) -> Option<[Option<f64>; 3]> {
    let mut groups: [Option<f64>; 3] = [None; 3];
    let mut found = 0_usize;
    let mut start: Option<usize> = None;

    for (index, character) in body.char_indices() {
        if character.is_ascii_digit() || character == '.' {
            start.get_or_insert(index);
        } else if matches!(character, '°' | '\'' | '"' | '′' | '″' | ' ' | ':' | '\t') {
            if !flush(body, &mut start, index, &mut groups, &mut found) {
                return None;
            }
        } else {
            // Anything else (stray letter, second sign) is not a number.
            return None;
        }
    }
    if !flush(body, &mut start, body.len(), &mut groups, &mut found) {
        return None;
    }

    (found > 0).then_some(groups)
}

/// Reads the group ending at `end`, if open, into the next slot.
fn flush(
    body: &str,
    start: &mut Option<usize>,
    end: usize,
    groups: &mut [Option<f64>; 3],
    found: &mut usize,
) -> bool {
    let Some(begin) = start.take() else {
        return true;
    };
    let Some(text) = body.get(begin..end) else {
        return false;
    };
    let Ok(value) = text.parse::<f64>() else {
        return false;
    };
    let Some(slot) = groups.get_mut(*found) else {
        return false;
    };
    *slot = Some(value);
    // `found` indexed `groups` just before, so it is in range.
    *found = found.saturating_add(1);
    true
}

/// Degrees, minutes and seconds to degrees.
fn combine(groups: &[Option<f64>; 3]) -> Option<f64> {
    let degrees = groups.first().copied().flatten()?;
    let minutes = groups.get(1).copied().flatten();
    let seconds = groups.get(2).copied().flatten();

    if !degrees.is_finite() {
        return None;
    }
    // Only the last group may have a fraction: `50°45.3'` is valid,
    // `50.5°45.3'` is not.
    if minutes.is_some() && !crate::math::is_integral(degrees) {
        return None;
    }

    let mut total = degrees;
    if let Some(minutes) = minutes {
        if !(0.0..60.0).contains(&minutes) {
            return None;
        }
        if seconds.is_some() && !crate::math::is_integral(minutes) {
            return None;
        }
        total += minutes / 60.0;
    }
    if let Some(seconds) = seconds {
        if !(0.0..60.0).contains(&seconds) {
            return None;
        }
        total += seconds / 3600.0;
    }

    total.is_finite().then_some(total)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::indexing_slicing)]
mod tests {
    use super::*;

    #[test]
    fn decimal_degrees() {
        let parsed = sexagesimal("latitude", "50.755").unwrap();
        assert_eq!(parsed.magnitude, 50.755);
        assert_eq!(parsed.hemisphere, None);
        assert!(!parsed.negative);
        assert_eq!(parsed.signed("S"), 50.755);
    }

    #[test]
    fn degrees_and_minutes() {
        for input in ["50°45.3'", "50 45.3", "50:45.3", "50°45.3′"] {
            let parsed = sexagesimal("latitude", input).unwrap();
            assert!((parsed.magnitude - 50.755).abs() < 1e-12, "{input}");
        }
    }

    #[test]
    fn degrees_minutes_and_seconds() {
        for input in ["50°45'18\"", "50 45 18", "50:45:18"] {
            let parsed = sexagesimal("latitude", input).unwrap();
            assert!((parsed.magnitude - 50.755).abs() < 1e-12, "{input}");
        }
    }

    #[test]
    fn hemispheres_lead_or_trail() {
        for input in ["50°45.3'N", "N50°45.3'", "n 50 45.3", "50 45.3 n"] {
            let parsed = sexagesimal("latitude", input).unwrap();
            assert_eq!(parsed.hemisphere, Some('N'), "{input}");
            assert!((parsed.signed("S") - 50.755).abs() < 1e-12);
        }
        let south = sexagesimal("latitude", "50°45.3'S").unwrap();
        assert!((south.signed("S") + 50.755).abs() < 1e-12);
    }

    #[test]
    fn signs_work_where_hemispheres_are_absent() {
        let negative = sexagesimal("longitude", "-1 17.8").unwrap();
        assert!(negative.negative);
        assert!((negative.signed("W") + 1.296_666_667).abs() < 1e-9);
        assert!(sexagesimal("longitude", "+1 17.8").unwrap().magnitude > 0.0);
    }

    #[test]
    fn nonsense_is_refused() {
        for input in [
            "",
            "   ",
            "north",
            "50 45.3 NW",
            "-50 45.3 N",  // a sign and a hemisphere disagreeing
            "50 60.0",     // sixty minutes is the next degree
            "50 45 60",    // and sixty seconds the next minute
            "50 45 18 12", // one group too many
            "50.5 45.3",   // fractional degrees with minutes as well
            "50 45.5 18",  // fractional minutes with seconds as well
            "50°45.3'W'N",
            "fifty",
            "50,45",
            "1e400",
        ] {
            assert!(
                sexagesimal("latitude", input).is_err(),
                "{input} should not parse"
            );
        }
        // Oversized input is rejected without parsing.
        let long = "1".repeat(200);
        assert!(sexagesimal("latitude", &long).is_err());
    }

    #[test]
    fn positions_split_at_the_hemisphere_letters() {
        for input in [
            "50°45.3'N 001°17.8'W",
            "50 45.3 N 001 17.8 W",
            "N50°45.3' W001°17.8'",
            "  50°45.3'N   001°17.8'W  ",
        ] {
            let (latitude, longitude) = split_position(input).unwrap();
            assert!(
                sexagesimal("latitude", latitude).is_ok(),
                "{input} -> {latitude:?}"
            );
            assert!(
                sexagesimal("longitude", longitude).is_ok(),
                "{input} -> {longitude:?}"
            );
        }
    }

    #[test]
    fn positions_split_on_a_separator_when_there_are_no_letters() {
        for input in ["50.755, -1.2967", "50.755 -1.2967", "50.755;-1.2967"] {
            let (latitude, longitude) = split_position(input).unwrap();
            assert!((sexagesimal("latitude", latitude).unwrap().magnitude - 50.755).abs() < 1e-9);
            assert!(sexagesimal("longitude", longitude).unwrap().negative);
        }
    }

    #[test]
    fn unsplittable_positions_are_refused() {
        for input in [
            "",
            "50.755",
            "50.755 -1.2967 extra",
            "W001°17.8' 50°45.3'N", // longitude first
            "50°45.3'N",
        ] {
            assert!(split_position(input).is_err(), "{input} should not split");
        }
    }
}
