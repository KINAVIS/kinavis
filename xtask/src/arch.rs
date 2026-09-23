//! Dependency rule check against the actual build graph.
//!
//! `ARCHITECTURE.md` requires dependencies to point inward: the kernel depends
//! on no workspace crate; adapters depend on the kernel, never on the
//! algorithms. This task enforces it in the build rather than in review.
//!
//! Two checks:
//!
//! 1. Every normal and build dependency edge of each workspace member in `cargo
//!    metadata` must be listed in `ci/allowed-deps.toml`, external crates
//!    included, so every new dependency is an explicit decision (as `deny.toml`
//!    also requires).
//! 2. No non-comment line of kernel source names a `kinavis` path. The crate
//!    graph already makes this fail to compile; this reports it earlier and
//!    more clearly.

use std::collections::BTreeMap;
use std::fmt;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

/// Allowed-edges file, relative to the workspace root.
const ALLOWED: &str = "ci/allowed-deps.toml";

/// Crate that must not be referenced from the kernel.
const OUTER_CRATE: &str = "kinavis";

/// Crates whose source must not reference the outer crate.
const INNER_CRATES: &[&str] = &["kinavis-kernel"];

/// Check finding or failure.
#[derive(Debug)]
pub(crate) struct Failure(String);

impl fmt::Display for Failure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl From<std::io::Error> for Failure {
    fn from(error: std::io::Error) -> Self {
        Self(error.to_string())
    }
}

impl From<serde_json::Error> for Failure {
    fn from(error: serde_json::Error) -> Self {
        Self(error.to_string())
    }
}

/// Workspace member from `cargo metadata`.
struct Member {
    name: String,
    /// Directory of its `Cargo.toml`.
    root: PathBuf,
    /// Normal and build dependency names (dev-dependencies excluded).
    shipped: Vec<String>,
}

/// Runs both checks and reports all violations.
pub(crate) fn run() -> Result<(), Failure> {
    let (workspace_root, members) = metadata()?;
    let allowed = allowed_edges(&workspace_root.join(ALLOWED))?;

    let mut violations = Vec::new();
    check_edges(&members, &allowed, &mut violations);
    check_kernel_source(&members, &mut violations)?;

    if violations.is_empty() {
        println!(
            "The dependency rule holds: {} crates, every edge listed in {ALLOWED}.",
            members.len()
        );
        return Ok(());
    }
    let mut report = String::from("The dependency rule is broken:\n");
    for violation in &violations {
        report.push_str("\n  ");
        report.push_str(violation);
    }
    report.push_str("\n\nDependencies point inward only. See ARCHITECTURE.md.");
    Err(Failure(report))
}

/// Edges not allowed by the list, and members missing from it.
fn check_edges(members: &[Member], allowed: &BTreeMap<String, Vec<String>>, out: &mut Vec<String>) {
    for member in members {
        let Some(permitted) = allowed.get(&member.name) else {
            out.push(format!(
                "{} is not listed in {ALLOWED}; add a table for it",
                member.name
            ));
            continue;
        };
        for dependency in &member.shipped {
            if !permitted.contains(dependency) {
                out.push(format!(
                    "{} -> {} is not an allowed edge",
                    member.name, dependency
                ));
            }
        }
    }
    for name in allowed.keys() {
        if !members.iter().any(|member| &member.name == name) {
            out.push(format!(
                "{ALLOWED} lists {name}, which is not a workspace member"
            ));
        }
    }
}

/// Non-comment inner-crate source lines naming the outer crate.
fn check_kernel_source(members: &[Member], out: &mut Vec<String>) -> Result<(), Failure> {
    let needle = format!("{OUTER_CRATE}::");
    for member in members
        .iter()
        .filter(|member| INNER_CRATES.contains(&member.name.as_str()))
    {
        for file in rust_files(&member.root.join("src"))? {
            let text = fs::read_to_string(&file)?;
            for (index, line) in text.lines().enumerate() {
                let code = line.trim_start();
                if code.starts_with("//") || !code.contains(&needle) {
                    continue;
                }
                out.push(format!(
                    "{}:{} names `{OUTER_CRATE}::` from inside {}",
                    file.display(),
                    index + 1,
                    member.name
                ));
            }
        }
    }
    Ok(())
}

/// `.rs` files under a directory, recursively, in stable order.
fn rust_files(dir: &Path) -> Result<Vec<PathBuf>, Failure> {
    let mut files = Vec::new();
    let mut pending = vec![dir.to_path_buf()];
    while let Some(dir) = pending.pop() {
        for entry in fs::read_dir(&dir)? {
            let path = entry?.path();
            if path.is_dir() {
                pending.push(path);
            } else if path.extension().is_some_and(|extension| extension == "rs") {
                files.push(path);
            }
        }
    }
    files.sort();
    Ok(files)
}

/// Workspace root and members from `cargo metadata`.
fn metadata() -> Result<(PathBuf, Vec<Member>), Failure> {
    let cargo = std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into());
    let output = Command::new(cargo)
        .args(["metadata", "--format-version", "1", "--no-deps"])
        .output()?;
    if !output.status.success() {
        return Err(Failure(format!(
            "cargo metadata failed:\n{}",
            String::from_utf8_lossy(&output.stderr)
        )));
    }
    let json: serde_json::Value = serde_json::from_slice(&output.stdout)?;

    let root = json
        .get("workspace_root")
        .and_then(serde_json::Value::as_str)
        .ok_or_else(|| Failure("cargo metadata: no workspace_root".into()))?;
    let packages = json
        .get("packages")
        .and_then(serde_json::Value::as_array)
        .ok_or_else(|| Failure("cargo metadata: no packages".into()))?;

    let mut members = Vec::new();
    for package in packages {
        let name = string_field(package, "name")?;
        let manifest = string_field(package, "manifest_path")?;
        let dependencies = package
            .get("dependencies")
            .and_then(serde_json::Value::as_array)
            .ok_or_else(|| Failure(format!("cargo metadata: {name} has no dependencies array")))?;
        let mut shipped = Vec::new();
        for dependency in dependencies {
            // `kind` is null for normal dependencies, "dev" or "build"
            // otherwise.
            if dependency.get("kind").and_then(serde_json::Value::as_str) == Some("dev") {
                continue;
            }
            shipped.push(string_field(dependency, "name")?);
        }
        let root = Path::new(&manifest)
            .parent()
            .ok_or_else(|| Failure(format!("{manifest} has no parent directory")))?
            .to_path_buf();
        members.push(Member {
            name,
            root,
            shipped,
        });
    }
    members.sort_by(|a, b| a.name.cmp(&b.name));
    Ok((PathBuf::from(root), members))
}

fn string_field(value: &serde_json::Value, field: &str) -> Result<String, Failure> {
    value
        .get(field)
        .and_then(serde_json::Value::as_str)
        .map(str::to_owned)
        .ok_or_else(|| Failure(format!("cargo metadata: missing string field `{field}`")))
}

/// Reads `ci/allowed-deps.toml`.
///
/// Parses only the documented shape — `[crate]` tables with `allow = ["...",
/// ...]` — and rejects anything else, so the check cannot be weakened by
/// unsupported syntax. A full TOML parser would add a dependency tree; the
/// format is small enough not to need one.
fn allowed_edges(path: &Path) -> Result<BTreeMap<String, Vec<String>>, Failure> {
    let text = fs::read_to_string(path)
        .map_err(|error| Failure(format!("{}: {error}", path.display())))?;
    parse_allowed(&text, path)
}

/// Parser for [`allowed_edges`]; `path` is used only in error messages.
fn parse_allowed(text: &str, path: &Path) -> Result<BTreeMap<String, Vec<String>>, Failure> {
    let mut edges = BTreeMap::new();
    let mut current: Option<String> = None;

    for (index, raw) in text.lines().enumerate() {
        let line = raw.trim();
        let at = |what: &str| Failure(format!("{}:{}: {what}", path.display(), index + 1));

        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some(table) = line
            .strip_prefix('[')
            .and_then(|rest| rest.strip_suffix(']'))
        {
            let table = table.trim();
            if table.is_empty() || !table.chars().all(is_bare_key_char) {
                return Err(at("a table name must be a bare crate name"));
            }
            if edges.insert(table.to_owned(), Vec::new()).is_some() {
                return Err(at("a crate is listed twice"));
            }
            current = Some(table.to_owned());
            continue;
        }
        let Some((key, value)) = line.split_once('=') else {
            return Err(at("expected `[crate]` or `allow = [...]`"));
        };
        if key.trim() != "allow" {
            return Err(at("the only key a crate table may hold is `allow`"));
        }
        let Some(table) = &current else {
            return Err(at("`allow` outside any `[crate]` table"));
        };
        let list = value
            .trim()
            .strip_prefix('[')
            .and_then(|rest| rest.strip_suffix(']'))
            .ok_or_else(|| at("`allow` must be an array of strings"))?;
        let mut names = Vec::new();
        for item in list
            .split(',')
            .map(str::trim)
            .filter(|item| !item.is_empty())
        {
            let name = item
                .strip_prefix('"')
                .and_then(|rest| rest.strip_suffix('"'))
                .ok_or_else(|| at("every allowed crate must be a quoted name"))?;
            if name.is_empty() || !name.chars().all(is_bare_key_char) {
                return Err(at("not a crate name"));
            }
            names.push(name.to_owned());
        }
        match edges.get_mut(table) {
            Some(slot) if slot.is_empty() => *slot = names,
            _ => return Err(at("`allow` given twice for one crate")),
        }
    }
    Ok(edges)
}

/// Characters valid in a crate name.
fn is_bare_key_char(character: char) -> bool {
    character.is_ascii_alphanumeric() || character == '-' || character == '_'
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(text: &str) -> Result<BTreeMap<String, Vec<String>>, Failure> {
        parse_allowed(text, Path::new("allowed.toml"))
    }

    #[test]
    fn the_promised_shape_is_read() -> Result<(), Failure> {
        let edges =
            parse("# comment\n[kernel]\nallow = []\n\n[core]\nallow = [\"kernel\", \"serde\"]\n")?;
        assert_eq!(edges.get("kernel"), Some(&Vec::new()));
        assert_eq!(
            edges.get("core"),
            Some(&vec!["kernel".to_owned(), "serde".to_owned()])
        );
        Ok(())
    }

    #[test]
    fn anything_else_is_refused() {
        for text in [
            "[a]\nallow = [\"b\"]\n[a]\nallow = []\n",
            "allow = []\n",
            "[a]\ndeny = []\n",
            "[a]\nallow = \"b\"\n",
            "[a]\nallow = [b]\n",
            "[a.b]\nallow = []\n",
        ] {
            assert!(parse(text).is_err(), "accepted: {text:?}");
        }
    }

    #[test]
    fn an_edge_off_the_list_is_a_violation() {
        let members = vec![Member {
            name: "core".into(),
            root: PathBuf::new(),
            shipped: vec!["kernel".into(), "rand".into()],
        }];
        let mut allowed = BTreeMap::new();
        allowed.insert("core".to_owned(), vec!["kernel".to_owned()]);
        let mut out = Vec::new();
        check_edges(&members, &allowed, &mut out);
        assert_eq!(out, vec!["core -> rand is not an allowed edge".to_owned()]);
    }
}
