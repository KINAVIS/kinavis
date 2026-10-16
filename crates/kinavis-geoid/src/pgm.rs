//! PGM header of a geoid grid, read from untrusted bytes.
//!
//! Netpbm binary greymap (`P5`) as GeographicLib writes geoid grids: comment
//! lines `# Offset <metres>` and `# Scale <metres>` before the dimensions, a
//! maximum value of 65535, one whitespace byte, then the samples.

use kinavis_kernel::error::{Excerpt, KernelError, Result};

/// Largest width or height accepted: a 1″ global grid is 1 296 000 columns,
/// far beyond any published geoid grid, and keeps indices in `i32`.
pub(crate) const MAX_DIMENSION: usize = 1 << 20;

/// Maximum sample value of a 16-bit grid.
const MAX_SAMPLE: u32 = 65_535;

/// Longest header line accepted, bytes; published headers are under 1 KiB.
const MAX_HEADER: usize = 4096;

/// The embedded EGM96 15′ samples, past their header.
#[cfg(feature = "egm96")]
pub(crate) const EGM96_SAMPLES: &[u8] =
    match include_bytes!("../data/egm96-15.pgm").split_at_checked(EGM96_HEADER) {
        Some((_, samples)) => samples,
        None => &[],
    };

/// Header length of the embedded EGM96 file, written by `tools/egm96_pgm.py`.
#[cfg(feature = "egm96")]
pub(crate) const EGM96_HEADER: usize = 261;

/// The embedded EGM96 file, header included.
#[cfg(all(feature = "egm96", test))]
pub(crate) const EGM96_FILE: &[u8] = include_bytes!("../data/egm96-15.pgm");

/// Checked header and the sample bytes after it.
#[derive(Debug)]
pub(crate) struct Header<'a> {
    pub(crate) width: usize,
    pub(crate) height: usize,
    pub(crate) offset: f64,
    pub(crate) scale: f64,
    pub(crate) samples: &'a [u8],
}

impl<'a> Header<'a> {
    pub(crate) fn parse(bytes: &'a [u8]) -> Result<Self> {
        let mut rest = bytes;
        let magic = next_line(&mut rest)?;
        if magic != "P5" {
            return Err(malformed("PGM magic number P5", magic));
        }
        let mut offset = None;
        let mut scale = None;
        let dimensions = loop {
            let line = next_line(&mut rest)?;
            if let Some(comment) = line.strip_prefix('#') {
                let mut words = comment.split_ascii_whitespace();
                match (words.next(), words.next()) {
                    (Some("Offset"), Some(value)) => offset = Some(number(value)?),
                    (Some("Scale"), Some(value)) => scale = Some(number(value)?),
                    _ => {}
                }
            } else if !line.trim().is_empty() {
                break line;
            }
        };
        let mut words = dimensions.split_ascii_whitespace();
        let (Some(width), Some(height), None) = (words.next(), words.next(), words.next()) else {
            return Err(malformed("PGM width and height", dimensions));
        };
        let width = dimension(width)?;
        let height = dimension(height)?;
        if width < 2 || width % 2 != 0 {
            return Err(malformed("an even PGM width of at least 2", dimensions));
        }
        if height < 3 || height % 2 == 0 {
            return Err(malformed("an odd PGM height of at least 3", dimensions));
        }
        let maximum = next_line(&mut rest)?;
        if maximum.trim().parse::<u32>().ok() != Some(MAX_SAMPLE) {
            return Err(malformed("PGM maximum value 65535", maximum));
        }
        let offset = offset
            .filter(|offset: &f64| offset.is_finite())
            .ok_or_else(|| malformed("finite geoid grid offset", ""))?;
        let scale = scale
            .filter(|scale: &f64| scale.is_finite() && *scale > 0.0)
            .ok_or_else(|| malformed("positive geoid grid scale", ""))?;
        let required = width
            .checked_mul(height)
            .and_then(|cells| cells.checked_mul(2))
            .unwrap_or(usize::MAX);
        if rest.len() != required {
            return Err(KernelError::InsufficientData {
                found: rest.len(),
                required,
                context: "geoid grid sample bytes",
            });
        }
        Ok(Self {
            width,
            height,
            offset,
            scale,
            samples: rest,
        })
    }
}

/// Takes one line, without its newline, from the front of `rest`; a line
/// longer than [`MAX_HEADER`] is refused.
fn next_line<'a>(rest: &mut &'a [u8]) -> Result<&'a str> {
    let window = rest.get(..rest.len().min(MAX_HEADER)).unwrap_or(&[]);
    let Some(end) = window.iter().position(|&byte| byte == b'\n') else {
        return Err(malformed("PGM header line", ""));
    };
    let (line, after) = rest.split_at(end);
    *rest = after.get(1..).unwrap_or(&[]);
    core::str::from_utf8(line).map_err(|_| malformed("ASCII PGM header", ""))
}

fn number(text: &str) -> Result<f64> {
    text.parse::<f64>()
        .map_err(|_| malformed("geoid grid offset or scale", text))
}

fn dimension(text: &str) -> Result<usize> {
    text.parse::<usize>()
        .ok()
        .filter(|value| *value <= MAX_DIMENSION)
        .ok_or_else(|| malformed("PGM dimension", text))
}

fn malformed(what: &'static str, input: &str) -> KernelError {
    KernelError::Parse {
        what,
        input: Excerpt::new(input),
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::indexing_slicing, clippy::float_cmp)]
mod tests {
    use super::*;

    const HEADER: &[u8] = b"P5\n# Offset -108\n# Scale 0.003\n4 3\n65535\n";

    #[test]
    fn a_well_formed_header_is_read() {
        let mut bytes = [0_u8; HEADER.len() + 24];
        bytes[..HEADER.len()].copy_from_slice(HEADER);
        let header = Header::parse(&bytes).unwrap();
        assert_eq!((header.width, header.height), (4, 3));
        assert_eq!(header.offset, -108.0);
        assert_eq!(header.scale, 0.003);
        assert_eq!(header.samples.len(), 24);
    }

    #[test]
    fn malformed_headers_and_wrong_lengths_are_refused() {
        let cases: [&[u8]; 8] = [
            b"P6\n# Offset 0\n# Scale 1\n4 3\n65535\n",
            b"P5\n# Scale 1\n4 3\n65535\n",
            b"P5\n# Offset 0\n# Scale -1\n4 3\n65535\n",
            b"P5\n# Offset nan\n# Scale 1\n4 3\n65535\n",
            b"P5\n# Offset 0\n# Scale 1\n5 3\n65535\n",
            b"P5\n# Offset 0\n# Scale 1\n4 4\n65535\n",
            b"P5\n# Offset 0\n# Scale 1\n4 3\n255\n",
            b"P5\n# Offset 0\n# Scale 1\n4 3",
        ];
        for case in cases {
            let mut bytes = [0_u8; 128];
            bytes[..case.len()].copy_from_slice(case);
            assert!(
                Header::parse(&bytes[..case.len() + 24]).is_err(),
                "{case:?}"
            );
        }
        let mut short = [0_u8; HEADER.len() + 23];
        short[..HEADER.len()].copy_from_slice(HEADER);
        assert!(matches!(
            Header::parse(&short),
            Err(KernelError::InsufficientData {
                found: 23,
                required: 24,
                ..
            })
        ));
    }

    #[cfg(feature = "egm96")]
    #[test]
    fn the_embedded_header_has_the_recorded_length() {
        let header = Header::parse(EGM96_FILE).unwrap();
        assert_eq!(EGM96_FILE.len() - header.samples.len(), EGM96_HEADER);
        assert_eq!(header.samples, EGM96_SAMPLES);
    }
}
