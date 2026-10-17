//! `cargo xtask mavlink DEFINITIONS.xml [LABEL]`: the messages of
//! `kinavis-mavlink`, generated from a MAVLink XML dialect.
//!
//! Reads the dialect and its includes, takes the messages of [`MESSAGES`],
//! and writes `crates/kinavis-mavlink/src/messages.rs`: a struct per message
//! with its id, `CRC_EXTRA`, wire length, decoder and encoder, and the
//! `Message` union. Fields go on the wire sorted by type size, largest first,
//! extensions after them in declared order; `CRC_EXTRA` is the X.25 checksum
//! of the name and the base fields, as the MAVLink generator computes it.
//! `LABEL` (the dialect's source and commit) goes into the file's header.
//!
//! The parser reads exactly the XML MAVLink definitions use: elements with
//! double-quoted attributes, text with the five predefined entities,
//! comments; nothing else.

use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

use crate::arch::Failure;

/// Messages generated, in output order.
const MESSAGES: &[&str] = &[
    "HEARTBEAT",
    "GPS_RAW_INT",
    "ATTITUDE",
    "LOCAL_POSITION_NED",
    "GLOBAL_POSITION_INT",
    "MISSION_ITEM_INT",
    "SET_POSITION_TARGET_GLOBAL_INT",
    "GPS_RTCM_DATA",
];

/// Output, relative to the workspace root.
const OUTPUT: &str = "crates/kinavis-mavlink/src/messages.rs";

/// One field of a message.
#[derive(Debug, Clone)]
struct Field {
    /// Base type: `uint8_t`, `float`, `char`…
    base: String,
    /// Array length, if an array.
    array: Option<usize>,
    name: String,
    units: Option<String>,
    description: String,
    extension: bool,
}

#[derive(Debug, Clone)]
struct MessageDef {
    id: u32,
    name: String,
    description: String,
    fields: Vec<Field>,
}

pub(crate) fn run(args: &[String]) -> Result<(), Failure> {
    let Some(definitions) = args.first() else {
        return Err(Failure(
            "usage: cargo xtask mavlink DEFINITIONS.xml [LABEL]".into(),
        ));
    };
    let label = args.get(1).cloned().unwrap_or_else(|| definitions.clone());
    let mut messages = Vec::new();
    read_dialect(Path::new(definitions), &mut Vec::new(), &mut messages)?;
    let mut chosen = Vec::new();
    for name in MESSAGES {
        let message = messages
            .iter()
            .find(|m| m.name == *name)
            .ok_or_else(|| Failure(format!("{name} is not in {definitions}")))?;
        chosen.push(message.clone());
    }
    let code = generate(&chosen, &label);
    let root = workspace_root()?;
    let output = root.join(OUTPUT);
    fs::write(&output, code)?;
    // Formatted as `cargo fmt --check` expects it.
    let status = std::process::Command::new("rustfmt")
        .args(["--edition", "2021"])
        .arg(&output)
        .status()?;
    if !status.success() {
        return Err(Failure(format!("rustfmt failed on {OUTPUT}")));
    }
    println!("Wrote {OUTPUT}: {} messages from {label}.", chosen.len());
    Ok(())
}

fn workspace_root() -> Result<PathBuf, Failure> {
    let manifest = std::env::var_os("CARGO_MANIFEST_DIR")
        .ok_or_else(|| Failure("run through cargo xtask".into()))?;
    Path::new(&manifest)
        .parent()
        .map(Path::to_path_buf)
        .ok_or_else(|| Failure("xtask has no parent directory".into()))
}

/// Messages of a dialect file and, first, of its includes.
fn read_dialect(
    path: &Path,
    seen: &mut Vec<PathBuf>,
    out: &mut Vec<MessageDef>,
) -> Result<(), Failure> {
    let canonical = path.canonicalize()?;
    if seen.contains(&canonical) {
        return Ok(());
    }
    seen.push(canonical);
    let text = fs::read_to_string(path).map_err(|e| Failure(format!("{}: {e}", path.display())))?;
    let elements = parse(&text).map_err(|e| Failure(format!("{}: {e}", path.display())))?;
    let directory = path.parent().unwrap_or(Path::new("."));
    for element in &elements {
        if element.name == "include" {
            read_dialect(&directory.join(element.text.trim()), seen, out)?;
        }
    }
    for element in &elements {
        if element.name == "message" {
            out.push(message(element)?);
        }
    }
    Ok(())
}

fn message(element: &Element) -> Result<MessageDef, Failure> {
    let id = element
        .attribute("id")
        .and_then(|id| id.parse().ok())
        .ok_or_else(|| Failure("a message without a numeric id".into()))?;
    let name = element
        .attribute("name")
        .ok_or_else(|| Failure("a message without a name".into()))?
        .to_owned();
    let mut fields = Vec::new();
    let mut extension = false;
    let mut description = String::new();
    for child in &element.children {
        match child.name.as_str() {
            "description" => description = tidy(&child.text),
            "extensions" => extension = true,
            "field" => {
                let kind = child
                    .attribute("type")
                    .ok_or_else(|| Failure(format!("{name}: a field without a type")))?;
                let (base, array) = match kind.split_once('[') {
                    Some((base, rest)) => {
                        let length = rest
                            .trim_end_matches(']')
                            .parse()
                            .map_err(|_| Failure(format!("{name}: bad array type {kind}")))?;
                        (base.to_owned(), Some(length))
                    }
                    None => (kind.to_owned(), None),
                };
                let base = if base == "uint8_t_mavlink_version" {
                    "uint8_t".to_owned()
                } else {
                    base
                };
                fields.push(Field {
                    base,
                    array,
                    name: child
                        .attribute("name")
                        .ok_or_else(|| Failure(format!("{name}: a field without a name")))?
                        .to_owned(),
                    units: child.attribute("units").map(str::to_owned),
                    description: tidy(&child.text),
                    extension,
                });
            }
            _ => {}
        }
    }
    Ok(MessageDef {
        id,
        name,
        description,
        fields,
    })
}

/// Fields in wire order: base fields stably by type size, largest first, then
/// extensions as declared.
fn wire_order(fields: &[Field]) -> Vec<Field> {
    let mut base: Vec<Field> = fields.iter().filter(|f| !f.extension).cloned().collect();
    base.sort_by_key(|f| std::cmp::Reverse(size_of(&f.base)));
    base.extend(fields.iter().filter(|f| f.extension).cloned());
    base
}

fn size_of(base: &str) -> usize {
    match base {
        "uint64_t" | "int64_t" | "double" => 8,
        "uint32_t" | "int32_t" | "float" => 4,
        "uint16_t" | "int16_t" => 2,
        _ => 1,
    }
}

/// `CRC_EXTRA`: X.25 of `NAME `, then `type name ` of each base field in wire
/// order, with the array length as a byte after an array's.
fn crc_extra(message: &MessageDef) -> u8 {
    let mut crc = 0xFFFFu16;
    let mut add = |bytes: &[u8]| {
        for byte in bytes {
            crc = accumulate(crc, *byte);
        }
    };
    add(message.name.as_bytes());
    add(b" ");
    for field in wire_order(&message.fields).iter().filter(|f| !f.extension) {
        add(field.base.as_bytes());
        add(b" ");
        add(field.name.as_bytes());
        add(b" ");
        if let Some(length) = field.array {
            add(&[u8::try_from(length).unwrap_or(u8::MAX)]);
        }
    }
    let [low, high] = crc.to_le_bytes();
    low ^ high
}

fn accumulate(crc: u16, byte: u8) -> u16 {
    let [low, _] = crc.to_le_bytes();
    let mut tmp = byte ^ low;
    tmp ^= tmp << 4;
    let tmp = u16::from(tmp);
    (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
}

/// Rust type of a field.
fn rust_type(field: &Field) -> String {
    let scalar = scalar(&field.base);
    match field.array {
        Some(length) => format!("[{scalar}; {length}]"),
        None => scalar.to_owned(),
    }
}

fn scalar(base: &str) -> &'static str {
    match base {
        "uint64_t" => "u64",
        "int64_t" => "i64",
        "double" => "f64",
        "uint32_t" => "u32",
        "int32_t" => "i32",
        "float" => "f32",
        "uint16_t" => "u16",
        "int16_t" => "i16",
        "int8_t" => "i8",
        _ => "u8",
    }
}

fn field_name(name: &str) -> String {
    match name {
        "type" => "mav_type".to_owned(),
        other => other.to_owned(),
    }
}

fn type_name(name: &str) -> String {
    name.split('_')
        .map(|part| {
            let mut chars = part.chars();
            chars.next().map_or_else(String::new, |first| {
                first.to_ascii_uppercase().to_string() + &chars.as_str().to_ascii_lowercase()
            })
        })
        .collect()
}

fn doc(text: &str, indent: &str) -> String {
    let escaped = text
        .replace('[', "\\[")
        .replace(']', "\\]")
        .replace('<', "&lt;")
        .replace('>', "&gt;");
    let mut out = String::new();
    let mut line = String::new();
    for word in escaped.split_whitespace() {
        if !line.is_empty() && line.len() + word.len() + 1 > 76 {
            let _ = writeln!(out, "{indent}/// {line}");
            line.clear();
        }
        if !line.is_empty() {
            line.push(' ');
        }
        line.push_str(word);
    }
    if !line.is_empty() {
        let _ = writeln!(out, "{indent}/// {line}");
    }
    if out.is_empty() {
        let _ = writeln!(out, "{indent}/// (No description.)");
    }
    out
}

fn generate(messages: &[MessageDef], label: &str) -> String {
    let mut out = String::new();
    let _ = writeln!(
        out,
        "//! MAVLink messages, generated by `cargo xtask mavlink` from {label}.\n//! Do not edit: regenerate.\n//!\n//! Every field is the wire value in the units of the definition; `crate::convert` turns\n//! them into kernel types. A payload shorter than the message (MAVLink 2 drops\n//! trailing zero bytes) reads as zeros beyond its end.\n\n#![allow(clippy::doc_markdown, clippy::struct_excessive_bools, clippy::too_many_lines)]\n\nuse crate::wire::{{Reader, Writer}};\n"
    );
    for message in messages {
        message_code(&mut out, message);
    }
    union_code(&mut out, messages);
    out
}

/// A message's struct, its constants and its codec.
fn message_code(out: &mut String, message: &MessageDef) {
    let rust = type_name(&message.name);
    let wire = wire_order(&message.fields);
    let length: usize = wire
        .iter()
        .map(|f| size_of(&f.base) * f.array.unwrap_or(1))
        .sum();
    out.push_str(&doc(
        &format!(
            "{} (#{}). {}",
            message.name, message.id, message.description
        ),
        "",
    ));
    let _ = writeln!(
        out,
        "#[derive(Debug, Clone, Copy, PartialEq)]\npub struct {rust} {{"
    );
    for field in &message.fields {
        let mut text = field.description.clone();
        if let Some(units) = &field.units {
            let _ = write!(text, " Units: {units}.");
        }
        if field.extension {
            text.push_str(" (Extension: zero from senders without it.)");
        }
        out.push_str(&doc(&text, "    "));
        let _ = writeln!(
            out,
            "    pub {}: {},",
            field_name(&field.name),
            rust_type(field)
        );
    }
    let _ = writeln!(out, "}}\n\nimpl Default for {rust} {{\n    fn default() -> Self {{\n        Self::ZERO\n    }}\n}}\n\nimpl {rust} {{\n    /// Message id.\n    pub const ID: u32 = {};\n    /// Checksum seed of the message's layout.\n    pub const CRC_EXTRA: u8 = {};\n    /// Payload length with every extension.\n    pub const LEN: usize = {length};\n    /// Every field zero.\n    pub const ZERO: Self = Self {{", message.id, crc_extra(message));
    for field in &message.fields {
        let zero = match (field.array, scalar(&field.base)) {
            (Some(length), "f32" | "f64") => format!("[0.0; {length}]"),
            (Some(length), _) => format!("[0; {length}]"),
            (None, "f32" | "f64") => "0.0".to_owned(),
            (None, _) => "0".to_owned(),
        };
        let _ = writeln!(out, "        {}: {zero},", field_name(&field.name));
    }
    let _ = writeln!(out, "    }};\n");
    let _ = writeln!(out, "    /// From a payload, zeros beyond its end.\n    #[must_use]\n    pub fn decode(payload: &[u8]) -> Self {{\n        let mut reader = Reader::new(payload);\n        let mut message = Self::ZERO;");
    for field in &wire {
        let name = field_name(&field.name);
        let scalar = scalar(&field.base);
        match field.array {
            Some(_) => {
                let _ = writeln!(out, "        for slot in &mut message.{name} {{\n            *slot = reader.{scalar}();\n        }}");
            }
            None => {
                let _ = writeln!(out, "        message.{name} = reader.{scalar}();");
            }
        }
    }
    let _ = writeln!(out, "        message\n    }}\n\n    /// Payload, every extension included; [`crate::Frame`] drops trailing\n    /// zeros on the way out.\n    #[must_use]\n    pub fn encode(&self) -> [u8; Self::LEN] {{\n        let mut payload = [0u8; Self::LEN];\n        let mut writer = Writer::new(&mut payload);");
    for field in &wire {
        let name = field_name(&field.name);
        let scalar = scalar(&field.base);
        match field.array {
            Some(_) => {
                let _ = writeln!(out, "        for value in self.{name} {{\n            writer.{scalar}(value);\n        }}");
            }
            None => {
                let _ = writeln!(out, "        writer.{scalar}(self.{name});");
            }
        }
    }
    let _ = writeln!(out, "        payload\n    }}\n}}\n");
}

/// `Message`, the union of the generated messages, and `crc_extra`.
fn union_code(out: &mut String, messages: &[MessageDef]) {
    let _ = writeln!(out, "/// A decoded message.\n///\n/// `#[non_exhaustive]`; match with a wildcard arm.\n#[non_exhaustive]\n#[derive(Debug, Clone, Copy, PartialEq)]\npub enum Message {{");
    for message in messages {
        let rust = type_name(&message.name);
        let _ = writeln!(
            out,
            "    /// {} (#{}).\n    {rust}({rust}),",
            message.name, message.id
        );
    }
    let _ = writeln!(out, "}}\n\nimpl Message {{\n    /// Decodes the payload of message `id`; `None` for an id not generated.\n    #[must_use]\n    pub fn decode(id: u32, payload: &[u8]) -> Option<Self> {{\n        Some(match id {{");
    for message in messages {
        let rust = type_name(&message.name);
        let _ = writeln!(
            out,
            "            {rust}::ID => Self::{rust}({rust}::decode(payload)),"
        );
    }
    let _ = writeln!(out, "            _ => return None,\n        }})\n    }}\n\n    /// Message id.\n    #[must_use]\n    pub const fn id(&self) -> u32 {{\n        match self {{");
    for message in messages {
        let rust = type_name(&message.name);
        let _ = writeln!(out, "            Self::{rust}(_) => {rust}::ID,");
    }
    let _ = writeln!(out, "        }}\n    }}\n\n    /// Payload into `out`, every extension included; returns its length.\n    pub fn encode_into(&self, out: &mut [u8; crate::MAX_PAYLOAD]) -> usize {{\n        let bytes: &[u8] = match self {{");
    for message in messages {
        let rust = type_name(&message.name);
        let _ = writeln!(out, "            Self::{rust}(m) => &m.encode(),");
    }
    let _ = writeln!(out, "        }};\n        for (slot, byte) in out.iter_mut().zip(bytes) {{\n            *slot = *byte;\n        }}\n        bytes.len()\n    }}\n}}\n\n/// `CRC_EXTRA` of a generated message id.\n#[must_use]\npub const fn crc_extra(id: u32) -> Option<u8> {{\n    match id {{");
    for message in messages {
        let rust = type_name(&message.name);
        let _ = writeln!(out, "        {rust}::ID => Some({rust}::CRC_EXTRA),");
    }
    let _ = writeln!(out, "        _ => None,\n    }}\n}}");
}

/// Collapses whitespace.
fn tidy(text: &str) -> String {
    text.split_whitespace().collect::<Vec<_>>().join(" ")
}

// ---------------------------------------------------------------- the XML

#[derive(Debug, Default)]
struct Element {
    name: String,
    attributes: Vec<(String, String)>,
    text: String,
    children: Vec<Element>,
}

impl Element {
    fn attribute(&self, name: &str) -> Option<&str> {
        self.attributes
            .iter()
            .find(|(key, _)| key == name)
            .map(|(_, value)| value.as_str())
    }
}

/// Top-level children of the root element.
fn parse(text: &str) -> Result<Vec<Element>, String> {
    let mut stack: Vec<Element> = vec![Element::default()];
    let mut rest = text;
    while let Some(open) = rest.find('<') {
        let (before, after) = rest.split_at(open);
        if let Some(top) = stack.last_mut() {
            top.text.push_str(&entities(before));
        }
        if let Some(comment) = after.strip_prefix("<!--") {
            let end = comment.find("-->").ok_or("an unclosed comment")?;
            rest = &comment[end + 3..];
            continue;
        }
        if let Some(declaration) = after.strip_prefix("<?") {
            let end = declaration.find("?>").ok_or("an unclosed declaration")?;
            rest = &declaration[end + 2..];
            continue;
        }
        let close = after.find('>').ok_or("an unclosed tag")?;
        let tag = &after[1..close];
        rest = &after[close + 1..];
        if let Some(name) = tag.strip_prefix('/') {
            let element = stack.pop().ok_or("a closing tag too many")?;
            if element.name != name.trim() {
                return Err(format!("</{}> closes <{}>", name.trim(), element.name));
            }
            stack
                .last_mut()
                .ok_or("a closing tag too many")?
                .children
                .push(element);
            continue;
        }
        let empty = tag.ends_with('/');
        let tag = tag.trim_end_matches('/');
        let (name, attributes) = tag.split_once(char::is_whitespace).unwrap_or((tag, ""));
        let element = Element {
            name: name.to_owned(),
            attributes: attributes_of(attributes)?,
            ..Element::default()
        };
        if empty {
            stack.last_mut().ok_or("no parent")?.children.push(element);
        } else {
            stack.push(element);
        }
    }
    let root = stack.pop().ok_or("no document")?;
    if !stack.is_empty() {
        return Err("unclosed elements".into());
    }
    // The document holds `<mavlink>`: its children are the definitions; the
    // messages and enums sit one level deeper.
    let mavlink = root
        .children
        .into_iter()
        .find(|e| e.name == "mavlink")
        .ok_or("no <mavlink>")?;
    let mut flat = Vec::new();
    for child in mavlink.children {
        if child.name == "messages" || child.name == "enums" {
            flat.extend(child.children);
        } else {
            flat.push(child);
        }
    }
    Ok(flat)
}

fn attributes_of(text: &str) -> Result<Vec<(String, String)>, String> {
    let mut attributes = Vec::new();
    let mut rest = text.trim();
    while !rest.is_empty() {
        let (key, after) = rest.split_once('=').ok_or("an attribute without a value")?;
        let after = after
            .trim_start()
            .strip_prefix('"')
            .ok_or("an unquoted attribute")?;
        let end = after.find('"').ok_or("an unclosed attribute")?;
        attributes.push((key.trim().to_owned(), entities(&after[..end])));
        rest = after[end + 1..].trim_start();
    }
    Ok(attributes)
}

fn entities(text: &str) -> String {
    text.replace("&lt;", "<")
        .replace("&gt;", ">")
        .replace("&quot;", "\"")
        .replace("&apos;", "'")
        .replace("&amp;", "&")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_heartbeat_has_the_published_crc_extra() {
        let fields = |specs: &[(&str, &str)]| {
            specs
                .iter()
                .map(|(base, name)| Field {
                    base: (*base).to_owned(),
                    array: None,
                    name: (*name).to_owned(),
                    units: None,
                    description: String::new(),
                    extension: false,
                })
                .collect()
        };
        let heartbeat = MessageDef {
            id: 0,
            name: "HEARTBEAT".into(),
            description: String::new(),
            fields: fields(&[
                ("uint8_t", "type"),
                ("uint8_t", "autopilot"),
                ("uint8_t", "base_mode"),
                ("uint32_t", "custom_mode"),
                ("uint8_t", "system_status"),
                ("uint8_t", "mavlink_version"),
            ]),
        };
        // 50, the value every MAVLink implementation carries.
        assert_eq!(crc_extra(&heartbeat), 50);
    }

    #[test]
    fn the_parser_reads_a_message() {
        let text = "<?xml version=\"1.0\"?><mavlink><!-- c --><messages><message id=\"7\" name=\"X\"><description>a &lt;b&gt;</description><field type=\"uint8_t[4]\" name=\"v\" units=\"m\">d</field><extensions/><field type=\"float\" name=\"w\">e</field></message></messages></mavlink>";
        let elements = parse(text).unwrap_or_default();
        let parsed = message(elements.first().unwrap_or(&Element::default())).ok();
        let parsed = parsed.unwrap_or(MessageDef {
            id: 0,
            name: String::new(),
            description: String::new(),
            fields: Vec::new(),
        });
        assert_eq!(parsed.id, 7);
        assert_eq!(parsed.description, "a <b>");
        assert_eq!(parsed.fields.len(), 2);
        assert_eq!(parsed.fields.first().and_then(|f| f.array), Some(4));
        assert!(parsed.fields.get(1).is_some_and(|f| f.extension));
    }
}
