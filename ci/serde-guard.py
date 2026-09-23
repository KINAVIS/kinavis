#!/usr/bin/env python3
"""Fails if a type with a numeric invariant can bypass it on deserialisation.

Value types validate in their constructors (latitude range, unit quaternion,
IMU interval). A derived `Deserialize` builds fields directly, bypassing the
check. Such types must deserialise through their constructor via
`#[serde(try_from = "Stored…")]`.

Syntactic check: a `pub struct` deriving `Deserialize` with a private field
holding a raw number (`f64`, integer, `bool`, or an array or `Option` of one)
must use `try_from`. Structs whose private fields are all validated types
(`Distance`, `Position`, `Instant`) pass. Structs holding raw numbers without
invariants are listed in `ALLOWED` with the reason.
"""

import re
import sys
from pathlib import Path

# Raw numbers without invariants: counts, indices, flags.
ALLOWED = {
    "GnssQuality": "satellites is a count; the DOPs are validated types",
    "SecondaryPort": "time differences in minutes, any value is a correction",
    "GuidanceView": "read model: a leg index and a flag",
    "MobDatum": "read model: two flags saying what was applied",
    "Alert": "read model: an occurrence count",
    "Turn": "read model: a waypoint index",
}

PRIMITIVE = re.compile(
    r"\b(f32|f64|i8|i16|i32|i64|i128|isize|u8|u16|u32|u64|u128|usize|bool)\b"
)
STRUCT = re.compile(r"((?:[ \t]*#\[[^\n]*\]\n)+)[ \t]*pub struct (\w+)[^{;]*\{")
FIELD = re.compile(r"^\s+(pub(?:\([^)]*\))?\s+)?([a-z_]\w*)\s*:\s*([^,\n]+)", re.M)


def check(path: Path) -> list[str]:
    text = path.read_text()
    # Test modules hold fixtures that are never deserialised.
    cut = text.find("#[cfg(test)]\nmod tests")
    if cut >= 0:
        text = text[:cut]
    problems = []
    for match in STRUCT.finditer(text):
        attributes, name = match.group(1), match.group(2)
        if "Deserialize" not in attributes or "try_from" in attributes:
            continue
        body_end = text.find("\n}", match.end())
        body = text[match.end() : body_end]
        private = [
            kind for visibility, _, kind in FIELD.findall(body) if not visibility
        ]
        if not any(PRIMITIVE.search(kind) for kind in private):
            continue
        if name in ALLOWED:
            continue
        line = text.count("\n", 0, match.start(2)) + 1
        problems.append(
            f"{path}:{line}: `{name}` derives Deserialize with a raw number in a "
            "private field and no `try_from`; read it back through its "
            "constructor, or list it in ALLOWED with the reason"
        )
    return problems


def main() -> int:
    problems = []
    for path in sorted(Path("crates").glob("*/src/**/*.rs")):
        problems.extend(check(path))
    for problem in problems:
        print(problem)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
