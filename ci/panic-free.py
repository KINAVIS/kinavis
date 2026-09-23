#!/usr/bin/env python3
"""Fails if the bare-metal build has a panic path outside `Display`.

A panic aborts the navigation process. The crates avoid every panic source
(no `unwrap`/`expect`/indexing, bounded loops, `Result` for fallible steps);
this script verifies it by reading the LLVM IR of the bare-metal build and
attributing every `core::panicking` call to its calling function.

Crates are checked individually: `--emit=llvm-ir` covers only the requested
package, and a call into a non-inlined kernel function appears as a bare
`declare`, with its body in the kernel's IR.

Exception: formatting implementations. `core::fmt` itself contains a bounds
check reached by runtime width or precision specifiers, which the crates
cannot remove.

Limitation: public functions small enough for cross-crate inlining stay as
MIR and are not covered here; they are one-liners, covered at source level by
the denied `indexing_slicing`, `unwrap_used`, `expect_used` and `panic` lints.
"""

import os
import re
import subprocess
import sys
from pathlib import Path

TARGET = "thumbv7em-none-eabihf"
FEATURES = "libm"
# Release build, and release with overflow checks: an unchecked wrapping `+`
# is a panic path in the latter and a silently wrong value in the former.
PROFILES = ["release", "strict"]
# All shipped crates, innermost first.
PACKAGES = ["kinavis-kernel", "kinavis", "kinavis-nmea0183", "kinavis-wmm", "kinavis-traffic", "kinavis-colregs", "kinavis-alerts", "kinavis-ais", "kinavis-nmea2000", "kinavis-ins"]
PANIC = re.compile(r"core9panicking")
DEFINE = re.compile(r"^define\b.*?@(\S+?)\(")


def emit_ir(package: str, profile: str) -> Path:
    out = Path("target") / TARGET / profile / "deps"
    for stale in out.glob("*.ll"):
        stale.unlink()
    # A cached build emits no IR: clean first, with the same profile and
    # target as the build below.
    subprocess.run(
        ["cargo", "clean", "-p", package, "--profile", profile, "--target", TARGET],
        check=True,
    )
    subprocess.run(
        [
            "cargo", "rustc", "--package", package,
            "--profile", profile, "--target", TARGET,
            "--no-default-features", "--features", FEATURES, "--lib",
            "--", "--emit=llvm-ir", "-C", "debuginfo=0", "-C", "codegen-units=1",
        ],
        check=True,
        # Incremental artefacts would suppress IR emission.
        env={**os.environ, "CARGO_INCREMENTAL": "0"},
    )
    files = sorted(out.glob("*.ll"))
    if len(files) != 1:
        sys.exit(f"expected one IR file in {out}, found {len(files)}")
    return files[0]


FMT_METHOD = re.compile(r"3fmt(B[0-9A-Za-z]*_)?$")


def is_formatting(symbol: str) -> bool:
    """Whether a mangled name is a `core::fmt` trait implementation.

    In the v0 mangling a generic instantiation — `Display for
    Direction<True>` — ends in a back-reference after the method name, so
    the suffix is matched rather than the bare `3fmt`.
    """
    return "4core3fmt" in symbol and FMT_METHOD.search(symbol) is not None


def check(package: str, profile: str) -> int:
    ir = emit_ir(package, profile)
    offenders: dict[str, int] = {}
    current = ""
    for line in ir.read_text().splitlines():
        found = DEFINE.match(line)
        if found:
            current = found.group(1)
        elif PANIC.search(line) and not line.lstrip().startswith("declare"):
            if not is_formatting(current):
                offenders[current] = offenders.get(current, 0) + 1

    if offenders:
        print(f"A panic path survives in the {profile} profile of {package}:\n")
        for symbol, count in sorted(offenders.items()):
            print(f"  {count:3}  {symbol}")
        print("\nUse checked access and return an error instead.")
        return 1

    print(f"No panic path outside formatting in {ir.name} ({profile}).")
    return 0


def main() -> int:
    return max(check(package, profile) for profile in PROFILES for package in PACKAGES)


if __name__ == "__main__":
    sys.exit(main())
