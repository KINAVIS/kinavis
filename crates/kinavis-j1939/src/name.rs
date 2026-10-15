//! NAME: the 64-bit identity a node claims its address with.

use core::fmt;

use crate::error::J1939Error;

/// NAME of a control function, ISO 11783-5 / J1939-81.
///
/// ```text
/// bit 63     62..60   59..56    55..49   48   47..40   39..35   34..32  31..21  20..0
///   self-   industry  class    device   res.  function function ECU    manu-   identity
///   config. group     instance class          instance        instance facturer number
/// ```
///
/// A lower NAME has the higher priority: in an address conflict it keeps the
/// address. Sent on the bus as 8 little-endian bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Name(u64);

/// Fields of a [`Name`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct NameFields {
    /// Whether the node may take another address on a conflict.
    pub self_configurable: bool,
    /// Industry group, 3 bits (2: agriculture and forestry).
    pub industry_group: u8,
    /// Device class instance, 4 bits.
    pub device_class_instance: u8,
    /// Device class, 7 bits.
    pub device_class: u8,
    /// Function, 8 bits.
    pub function: u8,
    /// Function instance, 5 bits.
    pub function_instance: u8,
    /// ECU instance, 3 bits.
    pub ecu_instance: u8,
    /// Manufacturer code, 11 bits.
    pub manufacturer_code: u16,
    /// Identity number, 21 bits.
    pub identity_number: u32,
}

impl Name {
    /// NAME from its 64 bits.
    #[must_use]
    pub const fn from_raw(raw: u64) -> Self {
        Self(raw)
    }

    /// NAME from its fields.
    ///
    /// # Errors
    ///
    /// [`J1939Error::BadName`] for a field wider than its bits.
    pub fn new(fields: NameFields) -> Result<Self, J1939Error> {
        let mut raw = u64::from(fields.self_configurable) << 63;
        for (field, value, bits, shift) in [
            ("industry group", u32::from(fields.industry_group), 3, 60),
            (
                "device class instance",
                u32::from(fields.device_class_instance),
                4,
                56,
            ),
            ("device class", u32::from(fields.device_class), 7, 49),
            ("function", u32::from(fields.function), 8, 40),
            (
                "function instance",
                u32::from(fields.function_instance),
                5,
                35,
            ),
            ("ECU instance", u32::from(fields.ecu_instance), 3, 32),
            (
                "manufacturer code",
                u32::from(fields.manufacturer_code),
                11,
                21,
            ),
            ("identity number", fields.identity_number, 21, 0),
        ] {
            if value >> bits != 0 {
                return Err(J1939Error::BadName { field, value });
            }
            raw |= u64::from(value) << shift;
        }
        Ok(Self(raw))
    }

    /// The 64 bits.
    #[must_use]
    pub const fn raw(self) -> u64 {
        self.0
    }

    /// NAME from the 8 little-endian bytes of an address claim.
    #[must_use]
    pub const fn from_bytes(bytes: [u8; 8]) -> Self {
        Self(u64::from_le_bytes(bytes))
    }

    /// The 8 little-endian bytes of an address claim.
    #[must_use]
    pub const fn to_bytes(self) -> [u8; 8] {
        self.0.to_le_bytes()
    }

    /// Fields.
    #[must_use]
    pub const fn fields(self) -> NameFields {
        const fn bits(raw: u64, shift: u32, width: u32) -> u64 {
            (raw >> shift) & ((1 << width) - 1)
        }
        let raw = self.0;
        // Each value is masked to its width, at most 21 bits.
        #[allow(clippy::cast_possible_truncation)]
        NameFields {
            self_configurable: self.0 >> 63 != 0,
            industry_group: bits(raw, 60, 3) as u8,
            device_class_instance: bits(raw, 56, 4) as u8,
            device_class: bits(raw, 49, 7) as u8,
            function: bits(raw, 40, 8) as u8,
            function_instance: bits(raw, 35, 5) as u8,
            ecu_instance: bits(raw, 32, 3) as u8,
            manufacturer_code: bits(raw, 21, 11) as u16,
            identity_number: bits(raw, 0, 21) as u32,
        }
    }

    /// Whether the node may take another address on a conflict.
    #[must_use]
    pub const fn is_self_configurable(self) -> bool {
        self.0 >> 63 != 0
    }
}

impl fmt::Display for Name {
    /// Formats as `NAME 0x200234000430744d`, sixteen hexadecimal digits.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "NAME {:#018x}", self.0)
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    #[test]
    fn a_claim_from_the_bus_reads_back_into_its_fields() {
        // An address claim of a John Deere tractor: 4D 74 30 04 00 34 02 20.
        let name = Name::from_bytes([0x4D, 0x74, 0x30, 0x04, 0x00, 0x34, 0x02, 0x20]);
        let fields = name.fields();
        assert!(!fields.self_configurable);
        assert_eq!(fields.industry_group, 2);
        assert_eq!(fields.device_class, 1);
        assert_eq!(fields.function, 0x34);
        assert_eq!(fields.manufacturer_code, 0x21);
        assert_eq!(fields.identity_number, 0x10_744D);
        assert_eq!(std::format!("{name}"), "NAME 0x200234000430744d");
        assert_eq!(Name::new(fields).unwrap(), name);
        assert_eq!(
            name.to_bytes(),
            [0x4D, 0x74, 0x30, 0x04, 0x00, 0x34, 0x02, 0x20]
        );
    }

    #[test]
    fn a_field_too_wide_is_refused() {
        let fields = NameFields {
            function_instance: 32,
            ..NameFields::default()
        };
        assert_eq!(
            Name::new(fields),
            Err(J1939Error::BadName {
                field: "function instance",
                value: 32
            })
        );
    }
}
