//! Signal K timestamps: RFC 3339 in UTC, `2024-01-15T12:30:00.000Z`.

use kinavis_kernel::time::{Civil, Instant, Utc};

/// Instant of a Signal K timestamp; `None` if it is not one.
///
/// Accepts a `Z` or `+00:00` suffix and zero to nine decimals of a second.
/// Other offsets are refused: the specification requires UTC.
#[must_use]
pub fn parse_timestamp(text: &str) -> Option<Instant<Utc>> {
    let text = text
        .strip_suffix('Z')
        .or_else(|| text.strip_suffix("+00:00"))?;
    let (date, time) = text.split_once('T')?;
    let mut date = date.split('-');
    let year = date.next()?.parse().ok()?;
    let month = date.next()?.parse().ok()?;
    let day = date.next()?.parse().ok()?;
    if date.next().is_some() {
        return None;
    }
    let (clock, fraction) = time.split_once('.').unwrap_or((time, ""));
    let mut clock = clock.split(':');
    let hour = clock.next()?.parse().ok()?;
    let minute = clock.next()?.parse().ok()?;
    let second = clock.next()?.parse().ok()?;
    if clock.next().is_some() {
        return None;
    }
    let nanos = nanos(fraction)?;
    Instant::from_civil(Civil {
        year,
        month,
        day,
        hour,
        minute,
        second,
        nanos,
    })
    .ok()
}

/// Nanoseconds of a decimal fraction of a second, at most nine digits.
fn nanos(fraction: &str) -> Option<u32> {
    if fraction.len() > 9 || !fraction.bytes().all(|byte| byte.is_ascii_digit()) {
        return None;
    }
    fraction
        .bytes()
        .chain(core::iter::repeat(b'0'))
        .take(9)
        .try_fold(0_u32, |nanos, digit| {
            nanos
                .checked_mul(10)?
                .checked_add(u32::from(digit.wrapping_sub(b'0')))
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_signal_k_timestamp_is_read_to_the_nanosecond() {
        let at = parse_timestamp("2024-01-15T12:30:00.250Z");
        assert_eq!(
            at.map(|at| at.to_string()),
            Some("2024-01-15T12:30:00.250 UTC".to_owned())
        );
        let whole = parse_timestamp("2024-01-15T12:30:00+00:00");
        assert_eq!(
            whole.map(|at| at.to_string()),
            Some("2024-01-15T12:30:00.000 UTC".to_owned())
        );
    }

    #[test]
    fn anything_else_is_not_a_timestamp() {
        for text in [
            "",
            "2024-01-15",
            "2024-01-15T12:30:00",
            "2024-01-15T12:30:00+02:00",
            "2024-02-30T12:30:00Z",
            "2024-01-15T12:30:00.1234567890Z",
            "2024-01-15T12:30:00.x1Z",
            "2024-01-15-01T12:30:00Z",
        ] {
            assert_eq!(parse_timestamp(text), None, "{text}");
        }
    }
}
