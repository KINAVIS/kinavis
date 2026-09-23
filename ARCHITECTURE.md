# KINAVIS architecture

A short map for people changing the code: the principles, the layers and the
rule between them, the bounded contexts, and what in the build holds it all
in place.

## Principles

- **A library, not a runtime.** No threads, no I/O, no "now", no global
  state. Everything that brings nondeterminism — clocks, the network, files,
  scheduling, models of the environment — comes in through ports declared in
  the kernel and implemented outside it. A step of any algorithm is therefore
  fully described by its inputs and outputs, and replays exactly when a
  voyage is investigated.
- **Events are returned, not broadcast.** An operation returns an
  `EventList` alongside its result; there are no callbacks. A callback would
  need an allocation, would get in the way of replay, and would see the state
  in the middle of an update. Bridging to queues and buses is the
  application's business.
- **Writes are separate from reads.** An aggregate (`Estimator`, `Traffic`,
  `AlertManager`) changes only through its own methods; what leaves it are
  projections — `NavigationSnapshot`, `GuidanceView`, `TrafficView`,
  `RouteEstimate` — that can be logged, sent to another thread and compared
  with a reference without exposing the filter's internals. There is no
  covariance in the public API, only an error ellipse.
- **A fact is separate from how alarming it is.** The domain says *what
  happened*; how alarming that is belongs to the vessel's policy
  (`AlertPolicy`, `CpaPolicy`, `ClearancePolicy`). Mathematics is separate
  from rules: CPA lives in `traffic`, the COLREGs in `colregs`. The state of
  the vessel is separate from the state of the environment.
- **External formats stay outside.** The adapters (NMEA 0183, NMEA 2000, AIS)
  are an anti-corruption layer: `Rmc` → `GnssFix` is a translation, not a
  reuse. The kernel is a shared kernel that changes only by extension; the
  estimator supplies guidance and traffic, which consume its read model, not
  its state.
- **Composition, not hierarchies.** A trait is a role (`Observation`,
  `ProcessModel`, `CompassModel`); no trait hierarchy is deeper than one
  level, and behaviour is extended with wrapper types. The public API never
  makes a caller walk through several dots to reach a value:
  `snapshot.horizontal_error()`, not `state.covariance.position…`.
- **One concept, one name.** Generalise on the third repetition, not before.

## The dependency rule

Dependencies point inward only. An outer layer knows about an inner one,
never the other way round.

```text
L0  crates/kinavis-kernel            angle, units, position, geodesy, local, time,
                                     observation, gnss, state, snapshot, estimation,
                                     environment, event, error, inline, math, matrix
        ↑
L1  crates/kinavis                   sailings, deviation, navigation_solutions,
                                     dead_reckoning, fix, relative_motion, route,
                                     gnss_intake, estimator, observations,
                                     conditions, tides, sun, guidance, turning,
                                     schedule, clearance, composite, anchor, mob
        ↑
L2  adapters                         crates/kinavis-nmea0183, crates/kinavis-nmea2000,
                                     crates/kinavis-wmm, crates/kinavis-ins —
                                     separate crates, depending on the kernel only;
                                     crates/kinavis-ais — over the kernel and nmea0183
                                     (the message arrives inside a sentence)
L2' built on the algorithms          crates/kinavis-traffic, crates/kinavis-colregs
                                     — depend on the kernel and kinavis
L3  where the contexts meet          crates/kinavis-alerts — over the kernel, kinavis
                                     and traffic: the one place where the events of
                                     every context meet
```

The layers are crates. Nothing in L0 has an alternative implementation: there
is one unit of measurement, but more than one way to compute a distance. That
is the criterion for the split. `kinavis` re-exports the kernel under its own
paths (`kinavis::angle`, …), so a user sees one crate; an adapter depends on
`kinavis-kernel` directly and pulls in none of the algorithms.

`math`, `inline` and `matrix` are public in the kernel, because "every call
to a transcendental function goes through one place" and "nothing allocates"
are rules for the whole family of crates. The kernel speaks only its own
vocabulary: `KernelError` is about numbers, buffers, matrices and time,
`NavigationEvent` about fixes, sensors and integrity; the errors of the
algorithms (`kinavis::NavigationError`, with `Kernel(KernelError)` inside)
and the events of the contexts (`GuidanceEvent`, `ClearanceEvent`,
`AnchorEvent`, `TrafficEvent`) live with their owners. `linalg` stays
private to `kinavis`: only `deviation` and `fix` need it. The unchecked
constructors the algorithms use are hidden (`#[doc(hidden)]`) and are not
part of the kernel's public API.

The vessel's settings (`GuidanceConfig`, `TurnParameters`, `EstimatorConfig`,
`StatePriors`, `TrackingPolicy`, `CpaPolicy`, `ManoeuvreConstraints`) follow
"parse, don't validate": the fields are private and checked once, in the
constructor (`new`, or `standard()` + `with_*`; the error is `OutOfRange`);
the hot path (`guide`, `assess`, `avoid`, an estimator step) does not check
them again, and a file is read through the same constructor (`Stored…`).
Where an invalid state can be made unrepresentable, it is (`PermittedSides`:
`Either`/`Starboard`/`Port` — there is no "neither side"). The active leg of a
route is a `LegCursor` that the `Route` issues and that is moved along it
(`advance`), not a number the caller made up.

## Bounded contexts

| Context | Modules | What it covers |
|---|---|---|
| Directions and angles | `angle`, `units` | the reference frames of courses, range invariants; `RateOfTurn` |
| Place | `position` | latitude, longitude, geocentric direction |
| Figure of the Earth | `geodesy` | the ellipsoid (WGS 84, GRS 80, International 1924, Clarke 1866, Airy 1830, Krassowsky 1940, Bessel 1841, Australian National), height with its datum, ECEF ↔ geodetic point; `Datum` (WGS 84, NAD83, ED50, NAD27, OSGB36, Pulkovo 1942, Tokyo, DHDN, AGD66, SAD69 — EPSG parameters with their stated accuracy, or one's own from a chart note) with `to_wgs84`/`from_wgs84` through `Helmert` (7 parameters, both rotation sign conventions, an exact inverse) |
| Local frames | `local` | NED/ENU/Body, `Vector3<F, U>` with its frame and unit in the type, `LocalFrame` |
| Time | `time` | an instant with its time scale in the type, the calendar, the leap second port |
| Observation | `observation` | a value + an instant + a quality; the age is computed |
| Satellite fix | `gnss` | `GnssFix` — where, when, how it was obtained, DOP; knows nothing of the sentence format |
| Estimator ports | `estimation` | `Observation` (the source is a `SensorId`, not a string), `ProcessModel`; Jacobians by component name |
| Environment | `environment` | the ports `MagneticModel`, `CurrentModel`, `WindModel`, `TideModel`, `LeewayModel`, `CompassModel`; `MagneticField`, `Current`, `Wind`, height of tide above chart datum; `EnvironmentSample` — answers, not sources |
| State | `state`, `snapshot` | the estimator's aggregate behind its invariants; the read model for a display |
| Events | `event` | what happened — returned as an `EventList<E, N>` (the event type is the `Event` trait, the kernel's `NavigationEvent` by default; the capacity is a parameter, `MAX_EVENTS` by default; an operation that can report more returns a list of its own size), never broadcast; the events of a context belong to that context |
| Earth's magnetic field | crate `kinavis-wmm` | WMM2025 behind a feature; `Wmm` implements `MagneticModel`; outside its validity — `OutsideValidity`; checked against NOAA's 100 test values |
| Traffic | crate `kinavis-traffic` | `TargetObservation` (radar — bearing and range, AIS — position and COG/SOG), `TargetTrack` — an aggregate with a window of fixes (`MAX_TRACK_HISTORY`), least-squares smoothing, extrapolation, age; `Traffic<N>` — the aggregate of the picture (the capacity is a parameter, `MAX_TARGETS` by default; not `Copy`), `TrackingPolicy::new(..)` — the vessel's checked thresholds + `WhenFull` (`Refuse` / `EvictStalest` — when the picture is full, the target observed longest ago makes way, with a `TargetEvicted` event); `TrafficView<N>` — the read model; `sweep`/`assess_traffic` return an `EventList<N>` — no event is lost; the events `TargetAcquired`/`TargetLost`/`TargetEvicted`/`ObservationRejected`; the collision assessment: `assess` (CPA/TCPA, bearing at CPA, rate of change of bearing, bow crossing range, `CollisionRisk` by `CpaPolicy`), `assess_track`, `assess_traffic` → `CollisionPicture` + `CpaAlarm`; the avoiding manoeuvre: `avoid`/`avoid_all` over `course_for_cpa` with `ManoeuvreConstraints` (sides, the smallest "readily apparent" and the largest alteration, rate of turn → time to turn), searching across every target in the picture |
| COLREGs | crate `kinavis-colregs` | rules, not physics: `Situation` (two `Party`s — motion, heading, `VesselCategory`; `Contact`; `Visibility`; the wind for rule 12), `ColregsConfig` (overtaking sector 22.5°, "nearly reciprocal" ±6°), `rule_of_the_road` → `Ruling { Encounter, Responsibility, Rule, PermittedManoeuvre }`; rules 12–15, 17, 18, 19; 9 and 10 are not applied |
| Alerts | crate `kinavis-alerts` | the `Reportable` trait (event → the condition's `AlertKind` and `Ended`), implemented for `NavigationEvent`, `GuidanceEvent`, `ClearanceEvent`, `AnchorEvent`, `TrafficEvent`; the `AlertPolicy` port (`classify(&AlertKind)` → Caution/Warning/Alarm/EmergencyAlarm, `rectify_after`), `StandardPolicy`; `AlertManager` — one `Alert` per `AlertKind` (kind + target/source/sensor), states Active/Acknowledged/Rectified/normal as in BAM, repeats are aggregated, an ending event (`FixAcquired`, `TargetLost`, a healthy sensor, Nominal) clears the condition, and so does silence longer than `rectify_after`; `alerts()` in order of priority; `AlertChanges` with `overflowed`/`lost`; `MAX_ALERTS` — on a full board an alert of higher priority than the last standing one takes its place (`AlertChange::Dropped`), one that is not is `lost`; `ingest<E: Reportable>` takes a slice of events from any context |
| NMEA 0183 | crate `kinavis-nmea0183` | framing, checksum, RMC/GGA/GLL/VTG ↔ kernel types, `GnssFix`; `!AIVDM`/`!AIVDO` → `Vdm` (fragments, sequence, channel, armoured payload, fill bits) — unpacked in the AIS crate |
| AIS | crate `kinavis-ais` | `Bits` — unpacking the 6-bit armouring into a `MAX_MESSAGE_BITS` buffer, fields by offset and width, 6-bit text into an `InlineStr` (trailing `@` and spaces trimmed); `Assembler<N>` — reassembling fragments by (VDO/VDM, sequence, channel) in `N` slots (`MAX_ASSEMBLIES` by default) with expiry; when every slot is taken, a new assembly evicts the oldest instead of being refused; `Message::decode` → `PositionReport` (types 1/2/3 class A, 18/19 class B: `TargetId` from the MMSI, `NavigationStatus`, `Turn`, `Speed`, `Position`, COG/HDG as `TrueCourse`, UTC second, accuracy, RAIM; type 19 also the name, `ShipType`, `Dimensions`, `PositionFixingDevice`), `StaticAndVoyageData` (type 5: IMO, call sign, name, `ShipType` with category and hazardous cargo class, `Dimensions`, `Eta`, draught as a `Distance`, destination, DTE), `StaticDataReport` (type 24, parts A/B; mothership for MMSI 98…), `AidToNavigation` (type 21: `AidType` — IALA `Mark`/`Quadrant`, name with extension, position, off position, virtual); "not available" is `None`; other types — `Unsupported { kind }`; the caller builds the observation for traffic |
| NMEA 2000 | crate `kinavis-nmea2000` | `CanId` (29 bits: priority, `Pgn` with PDU1/PDU2 taken into account, source, destination), `Frame` ≤ 8 bytes, a fixed-capacity `Payload` of `MAX_PAYLOAD_BYTES` with little-endian fields and bit fields from the least significant bit; `Assembler<N>` — fast-packet reassembly by (PGN, source) in `N` slots (`MAX_ASSEMBLIES` by default) with expiry (750 ms), the oldest evicted when every slot is taken, the transport from `Pgn::transport()` or from the caller (`push_as`); `Message::decode` → `PositionRapidUpdate` (129025), `CourseAndSpeed` (129026, `Referenced` — true/magnetic), `GnssPosition` (129029, with a `GnssFix` from it), `VesselHeading` (127250, with `Deviation`/`Variation`), `WaterDepth` (128267), `AisPositionReport` (129038/129039); "not available" (all ones; minus one is an error) is `None`; others — `Unsupported { pgn }`; the CAN bus itself is outside the crate |
| Inertial navigation | crate `kinavis-ins` | `Quaternion` body→NED (Euler angles, rotation-vector integration, the `(I − [ψ×])C` correction, `misalignment_from`), `ImuSample` (ω rad/s, f m/s², interval), `ImuNoise` (ARW/VRW/bias walk; `mems()`, `tactical()`); `Strapdown` — mechanisation in NED (Earth rate, transport rate, Somigliana gravity, Coriolis), position as a NED offset from a `LocalFrame`; `InsFilter` — a closed-loop 15×15 error-state EKF (δp, δv, ψ, b_g, b_a), `predict(&ImuSample)`, `update_position/point/velocity/zero_velocity/heading` with `GatingPolicy` and `InsUpdate { nis, dof, accepted }`, exposing an ellipse and σ of height/velocity/heading/tilt, not a matrix; `InsMotion: ProcessModel` — the bridge to the six-state estimator (velocity and rate of turn from the INS, noise σ²t); proven by Monte Carlo NEES (15 dof) and NIS in `tests/consistency.rs` |
| The ship's magnetism | `deviation` | the deviation table, interpolation, the A–E model; the table, `InterpolatedTable` and `SmithCoefficients` implement the `CompassModel` port |
| Course conversions | `navigation_solutions` | `corrections` — corrections that cannot fail, and gyro error; `compass` — compass ↔ magnetic ↔ true through `&impl CompassModel`, the table-based `convert_*` being the same `*_by` plus a report; `solver` — the inverse problem `CC + δ(CC) = MC`; `current` — the current triangle. Resolving a vector along a direction happens in one place: `Direction::components` in the kernel |
| Sailings | `sailings`, `route` | rhumb line, great circle, geodesic, the route; `LegCursor` — the active leg, issued by the route |
| Dead reckoning | `dead_reckoning` | DR and estimated positions |
| Position fixing | `fix` | position lines, the cocked hat |
| Relative motion | `relative_motion` | CPA, radar plotting, the avoiding manoeuvre |
| Estimation | `estimator`, `observations` | EKF: pure `pure::predict/update/update_late` + the `Estimator` shell with its history of estimates and its integrity and sensor verdicts; `SteadyMotion`; the standard observations |
| Environment models | `conditions`, `tides` | implementations of the `environment` ports: `Constant<T>`, `Timetable<T, N>` (component-wise interpolation), `FixedLeeway`; height of tide by the rule of twelfths (`TidalCycle`), secondary ports (`SecondaryPort`), the tidal diamond as a `CurrentModel` (`TidalStream`) |
| The sun | `sun` | azimuth and altitude (Meeus ch. 25 / NOAA), noon, sunrise/sunset and the three twilights for the local day; checking the compass by the sun |
| GNSS intake | `gnss_intake` | use case: a stream of fixes → a position snapshot + events; the thresholds are the vessel's settings |
| Turning | `turning` | `TurnMode` (radius / rate of turn), `TurnParameters::new` (advance and transfer from the pilot card, checked on construction), `Turn` — the wheel-over point, the tangent, the end of the turn |
| Guidance | `guidance` | use case: snapshot + route + environment → `GuidanceView` (desired track, course to steer allowing for current and leeway, XTE, the next turn) + the events `WaypointReached`/`WheelOverReached`/`CrossTrackExceeded`; a pure function without state, the active leg a `LegCursor` held by the caller (issued by the `Route`, moved by `advance`); inside a turn XTE is not an alarm |
| Composite sailing | `composite` | `composite_sailing(from, to, limit)` → `CompositeSailing`: the great circle if it stays within the limiting latitude, otherwise an arc to the parallel, a run along it and an arc from it (`ParallelRun`, vertices by Bowditch, `cos DLo = tan φ / tan L`); `route(interval)` — rhumb legs |
| Anchor watch | `anchor` | `swinging_radius` (cable, depth at the hawse, antenna to bow), `anchor_position`, `AnchorWatch` with a tolerance → `AnchorView` + the `AnchorDragging` event; a pure check of the snapshot |
| Man overboard | `mob` | `ManOverboard::datum(&EnvironmentSample, now, wind_factor)` → `MobDatum`: drift with the whole of the current and a fraction of the wind speed (IAMSAR); a missing component is not applied, and the datum says so |
| Under-keel clearance | `clearance` | `Hull` (draught, block coefficient), `Waterway` (open water / canal / blockage factor), squat by Barrass, `ClearancePolicy` (a minimum and a fraction of the draught), `Clearance` — charted depth + tide − draught − squat against the policy; the `UnderKeelClearanceLow` event; the height of tide at which the clearance is exactly the policy's |
| Schedule | `schedule` | `RouteSchedule` — a speed for every leg, ETD, ETA at each waypoint; checked on construction, after which the times are values; `RouteEstimate` from a snapshot and a `GuidanceView`: distance and time to go, ETA at the speed made good and by the plan, ahead or behind, the speed to arrive on time |

A context boundary is a possible crate boundary. A type does not cross a
boundary without a conversion: that is why `Direction::relabel` is hidden
from the public API.

## What holds these rules

- `Cargo.toml` (`[workspace.lints]`) — `unsafe_code = "forbid"`,
  `missing_docs`, `unwrap_used`, `expect_used`, `panic`, `indexing_slicing`,
  `must_use_candidate` — all `deny`, for every crate.
- `cargo fuzz run …` (the CI job `fuzz-smoke`, nightly, a matrix of seven
  targets) — everything untrusted bytes enter: NMEA 0183 `parse`; AIS
  `decode` (bits → message) and `assemble` (lines → sentences → fragment
  reassembly with a clock that also runs backwards → message); NMEA 2000
  `decode` (the bytes of a group) and `frames` (records of CAN frames →
  fast-packet reassembly → message); serde `serde_route` and `serde_table`
  (JSON → `Route`/`DeviationTable`, writing back is a fixed point, the table
  interpolates on every course). The corpora are in each crate's
  `fuzz/corpus/`.
- `ci/panic-free.py` — reads the LLVM IR of every crate built for
  `thumbv7em-none-eabihf` and fails on any path to a panic outside
  formatting. It runs twice: the `release` profile (what ships) and `strict`
  (`release` + `overflow-checks`), in which an unchecked integer
  `+`/`*`/`<<` is a path to a panic; hence `checked_*`/`saturating_*` in the
  code, or a bound the compiler can see (`min`, a half-open range).
- [`SECURITY.md`](SECURITY.md) — the threat model: trust boundaries (bytes
  off the wire and off the disk are untrusted; syntactically valid
  observations are semi-trusted — the source is not authenticated, but the
  damage is bounded by the gate, the verdict on the source and the overflow
  policies; the integrator's settings are trusted), a table of threat →
  response → how it is checked, what is out of scope, the disclosure policy.
  Every row of the table points at a CI job or a test.
- `ci/serde-guard.py` — a type with a numeric invariant is read from a file
  only through its constructor (`#[serde(try_from = "Stored…")]`); a derive
  without it on a struct with a private numeric field fails the build.
- The bare-metal build without `alloc`: the absence of allocation is checked
  by the compiler, not by a test.
- `tests/footprint.rs` in every crate with a large aggregate — `size_of`
  against a budget (Traffic 16 KiB, Estimator 9 KiB, AlertManager 3.5 KiB,
  Route 2.5 KiB, DeviationTable 1.25 KiB; the snapshot 192 B; an event and an
  error 64 B, one cache line). Growth past a budget is a deliberate decision,
  made by changing the test and the table in the guide (`kinavis::guide`,
  "Memory"), not a surprise. The rule: a type over a kilobyte lives behind a
  reference or in a `static`, and a projection is what crosses a boundary;
  `Traffic` and `AlertManager` are deliberately not `Copy` (proven by a
  `compile_fail` doctest).
- `tests/determinism.rs` in `kinavis`, `-wmm`, `-ins`, `-traffic` — `std` and
  `libm` give the same numbers: a snapshot taken from the `std` build,
  checked in both configurations to 1e-13 (≈ 1e-15 in practice).
- Hygiene in the spirit of MISRA: no recursion, a reason next to every
  `#[allow]` and every discarded `Result`, protocol field scales as named
  constants (`nmea2000::fields::resolution`, `ais::fields::counts_per`).
- `tests/properties.rs`, `tests/reference_vectors.rs` — properties over the
  whole domain, and agreement with external references.
- CI itself is part of the supply chain: every `uses:` in
  `.github/workflows/ci.yml` is pinned to a full commit SHA (the tag in a
  comment), `cargo-fuzz` is installed at a fixed version, the workflow token
  is read-only (`permissions: contents: read`); Dependabot proposes updates
  (`.github/dependabot.yml`: actions and `Cargo.lock`, weekly, one PR per
  ecosystem), and CI decides whether the jump is safe.
- `cargo xtask arch` (the CI job `arch`) — the dependency rule as data: every
  edge of the graph from `cargo metadata`, external crates included, must be
  listed in [`ci/allowed-deps.toml`](ci/allowed-deps.toml), and no line
  of kernel code names `kinavis::`. A new dependency means editing that file
  first, then `Cargo.toml`.
