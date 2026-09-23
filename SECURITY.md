# Security

KINAVIS is a navigation library: it turns sensor readings, chart data and
a vessel's settings into positions, courses, collision assessments and
alerts. The worst thing it can do is not crash — it is to hand the officer of
the watch a confident wrong number. This document says what we consider a
vulnerability, how to report one, what the threat model is, and how the
claims below are checked rather than made.

## Reporting a vulnerability

Please do not open a public issue for a suspected vulnerability.

- Preferred: GitHub's private vulnerability reporting on the repository
  (**Security → Report a vulnerability**), where it is enabled.
- Otherwise: email the maintainer at the address in `Cargo.toml`
  (`authors`), with `[KINAVIS security]` in the subject.

Include the crate and version, the input that triggers the problem (a
sentence, a frame, a JSON document, a sequence of observations), and what
happened against what should have. A minimal reproduction as a test or a
fuzz-crash file is ideal; a description is enough.

You will get an acknowledgement within 7 days and a fix or a considered
answer within 90. Fixes ship as patch releases of every affected crate, with
a `RUSTSEC` advisory requested when the problem is reachable from untrusted
input. Reporters are credited in the advisory unless they prefer not to be.

## Supported versions

| Crate | Supported |
|---|---|
| `kinavis`, `kinavis-kernel` | the latest `1.x` |
| `kinavis-nmea0183` | the latest `0.2.x` |
| `kinavis-ais`, `kinavis-nmea2000`, `kinavis-ins`, `kinavis-traffic`, `kinavis-colregs`, `kinavis-alerts`, `kinavis-wmm` | the latest `0.x` |

Older releases are not patched; upgrade. The minimum supported Rust version
is stated in `Cargo.toml` (`rust-version`) and tested in CI.

## What counts as a vulnerability

Anything that breaks one of these promises on data the caller did not
control:

1. **No panic, no abort, no unbounded work.** Every public function returns a
   value or an error for every input, in bounded time and bounded memory.
   A panic, an infinite loop, a stack overflow or an out-of-bounds read
   reached from an NMEA sentence, a CAN frame, an AIS payload or a stored
   file is a vulnerability.
2. **No silently wrong number.** An arithmetic overflow, a non-finite value,
   a truncation or a unit mix-up that produces a plausible but wrong result
   is a vulnerability — a more serious one than a crash, because it is
   acted on.
3. **No invariant bypass.** Every type with an invariant (a latitude in
   range, a unit quaternion, a positive-definite covariance, a table that
   can be inverted) is constructed through a checked constructor, including
   when it is read back through `serde`. A way to obtain an invalid value
   of such a type without `unsafe` is a vulnerability.
4. **No hidden `unsafe`, no hidden allocation.** `unsafe_code` is forbidden
   in every crate; the `no_std` configuration has no allocator. Either
   appearing is a vulnerability in the build, not only in the code.

Not vulnerabilities, but welcome as ordinary issues: a wrong answer on a
*valid* input (a bug in the mathematics), a limit that is too small for a
real installation, a missing check on a *trusted* argument, performance.

## Threat model

### What is being protected

The integrity of the navigation solution and of what is shown or alarmed
from it: the position and its error ellipse, the course to steer, the
traffic picture and its collision assessments, the standing alerts. And the
availability of the process that computes them: a navigation task that
panics or hangs on a malformed sentence has failed the watch.

### Trust boundaries

```text
 untrusted                     the crates                          trusted
 ─────────                     ──────────                          ───────
 NMEA 0183 bytes ──▶ kinavis-nmea0183    ──▶ typed sentences ─┐
 AIS payloads    ──▶ kinavis-ais         ──▶ Message ─────────┤
 CAN frames      ──▶ kinavis-nmea2000    ──▶ Message ─────────┼──▶ observations ──▶ estimator, traffic ──▶ views, events ──▶ alerts
 stored files    ──▶ serde (try_from)    ──▶ Route, tables ───┘                 ▲
                                                                               │
                                              the vessel's settings, built in code by the integrator
```

- **Untrusted: bytes off the wire and off the disk.** Anything that arrives
  as bytes — NMEA 0183 sentences, NMEA 2000 frames, AIS payloads, JSON or
  any other `serde` form — may be malformed, truncated, oversized,
  contradictory or crafted. The parsers are the only place these enter, and
  they promise the four properties above on any byte sequence.
- **Semi-trusted: well-formed observations.** A syntactically valid fix,
  heading or target report may still be wrong or hostile: a spoofed GNSS
  position, a jammed receiver reporting garbage with a clear conscience, an
  AIS transponder inventing identities. The library does not authenticate
  sources (AIS and NMEA have no means to), but it bounds what one can do:
  the estimator gates each observation against its own prediction and
  keeps a health verdict per source; the traffic picture rejects
  implausible jumps and has a stated policy for a flood of new identities;
  the alert board is finite and keeps the most pressing.
- **Trusted: the integrator's code.** Configuration — gates, limits,
  policies, tracking thresholds — is built by the calling program through
  checked constructors and is not an attack surface in itself; a
  configuration that would make the vessel unsafe is refused where it can
  be recognised (a negative limit, a zero alert distance), not second-
  guessed where it cannot.

### Threats and what answers them

| Threat | Where it enters | What answers it | Checked by |
|---|---|---|---|
| Malformed or oversized sentence, frame or payload crashes the parser | nmea0183 `parse`, nmea2000 `decode`/`frames`, ais `decode`/`assemble` | Fixed-size buffers with published bounds (`MAX_SENTENCE_BYTES`, `MAX_PAYLOAD_BYTES`, `MAX_MESSAGE_BITS`); every length and index checked; errors carry at most `EXCERPT_BYTES` of the input | `cargo fuzz` targets for each entry point (job `fuzz-smoke`), `tests/robustness.rs`, `ci/panic-free.py` on the bare-metal IR |
| Integer overflow yields a wrong length, offset or time | every crate | `checked_*`/`saturating_*` or a bound the compiler can see | `ci/panic-free.py` on the `strict` profile (overflow checks on), so any unchecked arithmetic shows as a panic path |
| Non-finite or out-of-range number reaches a calculation | every constructor | `NaN`, infinities and out-of-range values refused at construction; every result type checks its own output | `tests/robustness.rs` and property tests over the whole domain |
| A stored file carries a value the constructor would refuse | every `serde` `Deserialize` | `#[serde(try_from = "Stored…")]` on every type with an invariant, so a file goes through the same checks as code | `ci/serde-guard.py` fails the build on a `Deserialize` derive that skips the constructor; `tests/serde_round_trip.rs` |
| Fragment flood holds the reassembly slots so real messages never complete | ais and nmea2000 `Assembler` | Bounded slots (`MAX_ASSEMBLIES`); a fragment that does not finish in time is dropped; when the slots are full the oldest assembly is evicted, never the newcomer refused forever | `tests/assembler.rs`, fuzz target `assemble`/`frames` with a clock that runs both ways |
| Identity flood fills the traffic picture and hides the real target | traffic `ingest` | `MAX_TARGETS` slots and a stated `WhenFull` policy: refuse the newcomer and say so, or evict the target least recently seen and report it — the integrator chooses, nothing is silent | `tests/robustness.rs` flood tests |
| Spoofed or wild position pulls the estimate | estimator `ingest` | Chi-square gating against the prediction; rejections reported as events; `suspect_after` rejections in a row mark the source `Suspect`; integrity drops to `Exceeded`/`DeadReckoning` and is reported | Monte-Carlo consistency tests (`tests/filter_consistency.rs`) including a 500 m spoof |
| Implausible target jump swaps tracks | traffic `ingest` | `max_speed` in the tracking policy: an observation the target could not have reached is rejected, with the target named | unit and robustness tests |
| Alert flood evicts the alarm that matters | alerts `ingest` | `MAX_ALERTS` standing, in order of priority; when the board is full a newcomer that outranks the least pressing alert takes its place and the drop is reported (`AlertChange::Dropped`), one that does not is not raised and `AlertChanges::lost` says so — the bridge always sees the most pressing | `tests/robustness.rs` |
| Unbounded loop on a degenerate input | solvers and iterations | Every loop has a named public bound (`MAX_ITERATIONS_*`, `MAX_BISECTIONS_*`, `MAX_DEGREE`); exceeding it is `NotConverged`, not a hang | enforced in review; the bound is named in each function's `# Errors` |
| Stack exhaustion from a large aggregate on a small target | every aggregate | Published size budgets (the `kinavis::guide`, *Memory*), the rule that anything over a kilobyte lives behind a reference or in a `static`; the largest are not `Copy` | `tests/footprint.rs` in each crate |
| A dependency turns malicious or vulnerable | the build | Zero dependencies by default, `libm` alone for `no_std`, `serde` optional; every edge of the dependency graph listed in `ci/allowed-deps.toml` | `cargo deny` bans, licenses and sources and `cargo xtask arch` on every change; `cargo deny` advisories on every change and weekly (`advisories.yml`) |
| A CI action or a build-time tool is replaced under a moving tag | the workflow | Every `uses:` pinned to a full commit SHA with the tag in a comment; `cargo-fuzz` installed at a fixed version; the workflow token is read-only (`permissions: contents: read`); Dependabot proposes the bumps and CI judges them | `.github/workflows/ci.yml`, `.github/dependabot.yml` |
| `unsafe` or an allocation slips in | the build | `#![forbid(unsafe_code)]` workspace-wide; `no_std` build has no `extern crate alloc` | The `thumbv7em-none-eabihf` build in CI would not link |

### Out of scope

- **Authentication of the data itself.** AIS, NMEA 0183 and NMEA 2000 carry
  no signature; a transponder may say what it likes. The library bounds the
  damage a lying source can do and reports the suspicion; deciding whom to
  believe is the integrator's and the watch's.
- **The transport and the host.** Serial lines, CAN controllers, the RTOS,
  the process that owns the memory: their integrity is assumed.
- **Timing side channels.** Nothing here is a secret; execution time may
  depend on the input.
- **Denial of service by volume.** The library processes one input in
  bounded time; how many inputs per second the host accepts is the host's
  to limit.
- **The correctness of the chart, the almanac and the magnetic model.**
  They are inputs. A wrong datum or a stale `WMM` gives a wrong answer with
  a clear conscience; the library can only refuse what it can recognise as
  impossible.

## How the claims are checked

Every claim in this document is a CI job or a test, not a policy:

| Claim | Check |
|---|---|
| No panic path on bare metal, with and without overflow checks | `ci/panic-free.py` reads the LLVM IR of the `release` and `strict` profiles for `thumbv7em-none-eabihf` |
| Parsers survive arbitrary bytes | `cargo fuzz` targets in `crates/*/fuzz`, run on every change (job `fuzz-smoke`) and seeded from the tests' corpora |
| `serde` cannot bypass a constructor | `ci/serde-guard.py` |
| No `unsafe`, no `unwrap`, no `panic!`, no indexing without a check | workspace `[lints]`: `unsafe_code = "forbid"`, `unwrap_used`, `expect_used`, `panic`, `indexing_slicing` denied |
| The dependency graph is exactly what is declared | `cargo xtask arch` against `ci/allowed-deps.toml`; `cargo deny` |
| The build runs the code it was reviewed with | actions pinned to commit SHAs, tools to versions, read-only token; Dependabot for the bumps |
| Results are the same on `std` and `libm` | `tests/determinism.rs` |
| Public API changes are versioned | `cargo semver-checks` against the last release on crates.io (job `semver`, on from the first release) |

## Advice for integrators

- Feed the parsers everything and trust nothing until it has a type; the
  `Err` you get back is the point, log it and move on.
- Read the events. A rejected observation, a suspect sensor, an evicted
  target and a dropped alert are all reported as values; a program that
  discards them has discarded the library's view of what is wrong.
- Choose `WhenFull` and `TrackingPolicy::max_speed` for your waters; the
  defaults are conservative, not universal.
- Size the stack from the *Memory* table in the guide (`kinavis::guide`), or put the
  aggregates in `static` storage.
- Pin the crate versions and let `cargo deny`/`cargo audit` run in your own
  CI: this library's advisories will reach you that way.
