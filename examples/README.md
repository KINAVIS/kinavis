# KINAVIS demonstrations

Six short programs, each run on data recorded from real equipment. Nothing
here is simulated except where it says so.

```sh
cargo run -p kinavis-examples --example receivers
cargo run -p kinavis-examples --example gnss_jump
cargo run -p kinavis-examples --example traffic
cargo run -p kinavis-examples --example course_to_steer
cargo run -p kinavis-examples --example integrity
cargo run -p kinavis-examples --example field
```

## `receivers` — what real receivers send

One sentence from each of seven receivers: a chartplotter, an RTK receiver
writing past the standard's 82 bytes, an AIS transponder that drops a field, a
receiver that writes 999.9 for an unknown variation, a cold start, a receiver
without a fix, and a line that lost bytes in transit.

```text
u-blox ZED-F9P, high-precision NMEA, 89 bytes (ublox-f9p-hpnmea.log)
  $GNGGA,014500.00,4404.1306024,N,12118.8446777,W,2,12,0.49,1129.913,M,-21.350,M,,0278*4C
  -> position 44°04.1306024'N 121°18.8446777'W (Differential, HDOP 0.49)

Caterpillar MS352, variation 999.9 for "unknown" (cat-ms352.log)
  $GPRMC,113938.50,A,3842.86006889,N,11705.43645510,W,0.011,46.405,231122,999.9000,E,D*2E
  -> fix 38°42.8601'N 117°05.4365'W at 2022-11-23T11:39:38 UTC, 0.0 kn over the ground on 046.4°T

GPS-320FW after a GPS week rollover, bytes lost mid-sentence (gp-320fw-2019-04-07-coldboot.log)
  $GPRMC,000429.00,V,0000.0000,N,00000.000VTG,,T,,M,,N,,K,N*2C
  -> refused: checksum 2C claimed, body sums to 65
```

## `gnss_jump` — a forty-mile jump is refused

A yacht's real track, one fix a second, with one fix moved forty miles north.

```text
2018-08-20T09:48:50 UTC  accepted  52°50.940'N 005°18.662'E
2018-08-20T09:48:51 UTC  REFUSED   53°30.939'N 005°18.660'E  implausible jump: 144094 kn implied; injected
2018-08-20T09:48:52 UTC  accepted  52°50.938'N 005°18.657'E

145 fixes accepted, 1 refused.
```

## `traffic` — AIS in, CPA and TCPA out

1459 AIS messages from a receiver off Harlingen, reassembled and decoded; one
ship taken as own ship, every other ship assessed against it.

```text
ship                     bearing    range      CPA    TCPA  risk
245513000                075.0°T  35.61 M   0.12 M   76:02  developing
218784000                147.9°T   1.34 M   1.33 M    0:55  DANGEROUS
246754000                062.7°T  25.80 M   2.03 M   45:33  passing clear
```

## `course_to_steer` — guidance along a real track

The same yacht against a passage plan drawn for the example: desired track,
course to steer, cross-track error, distance to go, and the cross-track alarm.

```text
UTC       position                       COG    track    steer        off track   to go  alarm
09:47:37  52°51.009'N 005°18.817'E   230.5°T  235.7°T  235.7°T      0.19 M Port  8.82 M  off track: 0.19 M > 0.10 M
```

## `integrity` — a u-blox receiver checked: integrity, grids, height

A NEO-M8P in Prague, UBX binary (`ublox-neo-m8p.log` from gpsd). Each epoch:
the fix with the receiver's own accuracy; RAIM on its satellite residuals with
the protection levels, for a single-frequency code receiver (3 m at the
zenith); the position on UTM, MGRS and СК-42; the height above mean sea level
by EGM96 beside the receiver's. In the third epoch a 40 m error is injected on
the highest satellite, and RAIM finds and excludes it.

```text
2021-06-10T18:18:55.5 UTC  Autonomous, 9 satellites, ±2.52 m (receiver)
  RAIM   consistent (test 2.07 ≤ 4.30), HPL 32.3 m, VPL 63.2 m
  UTM    33N 459260.65 E 5553805.82 N
  MGRS   33UVR5926053805
  СК-42  x 5556174.43  y 3459372.79  (zone 3)
  height 348.86 m above the ellipsoid, 303.88 m above mean sea level by EGM96, 304.59 m by the receiver

  (injected: 40 m on G03)
2021-06-10T18:18:56.0 UTC  Autonomous, 9 satellites, ±2.52 m (receiver)
  RAIM   FAULT (test 8.24 > 4.30), excluded G03, then HPL 44.6 m
```

The receiver's own height above sea level differs from EGM96 by 0.7 m.

## `field` — a tractor's passes read back: where the implement was

A Fendt 820 with a 4.2 m cultivator, deep cultivation near Munich: 28 minutes
of its log at 5 Hz, twelve passes back and forth (an excerpt of TUM's
*Agricultural Load Cycles*). The first pass is the AB line. From pass to pass
the logged position steps across alternately about 2.4 m and 5.3 m, so the
implement is not under it: the passes are spaced equally for the implement
0.74 m to the left. Pass by pass, the tracker follows the implement's place;
the coverage raster, laid out from the logged position, leaves a strip
unworked at every other seam, and from the implement's place almost none.
The headland turns back up once each.

```text
pass  course   length   offset   sd    step
   0      AB    267 m   +0.50  0.23
   1      BA    260 m   -1.95  0.33   -2.44
   2      AB    261 m   -7.27  0.16   -5.32
   3      BA    258 m   -9.63  0.41   -2.36

Spacing 3.98 m: the 4.2 m implement overlaps 0.22 m. The logged steps alternate because the implement works 0.74 m left of the logged position.

coverage              worked    twice    left unworked between passes
from the logged fix    11066 m²   2710 m²   1090 m of seam
from the implement     12920 m²    857 m²    153 m of seam

11 headland turns: 23 m and 19 s a turn, changing direction 2 times in each.
The shortest turn without reversing, at a 6 m radius: 36 m.
```

What is left unworked from the implement's place is where a pass strays
from its line by more than the 22 cm overlap.

## Every gpsd log, in CI

`tests/gpsd_corpus.rs` reads every sentence of the gpsd regression logs —
about two hundred logs of real receivers, fetched at a pinned gpsd commit —
and counts what became of each: read as which sentence, refused for which
error, a fix or not, which AIS message. CI fails when a count changes, so a
parser change shows its effect on real receivers in the diff of
[`tests/gpsd_corpus.expected`](tests/gpsd_corpus.expected):

```text
   2659 RMC: fix
    180 RMC: no fix, NoPosition
   3026 GGA: position
    804 AIS: position report, type 1
    388 refused: BadChecksum
    483 refused: NoChecksum
```

To run it locally, with gpsd's `test/daemon` checked out:

```sh
GPSD_LOGS=path/to/gpsd/test/daemon cargo test -p kinavis-examples --test gpsd_corpus -- --ignored
```

## Data

| File | Source | Licence |
|---|---|---|
| `data/receivers.nmea` | [gpsd](https://gitlab.com/gpsd/gpsd) regression logs, `test/daemon/`, one line from each of seven | BSD-2-Clause, [`data/LICENSE-gpsd`](data/LICENSE-gpsd) |
| `data/bundg_zeus_9.nmea` | gpsd `test/daemon/bundg_zeus_9.log`, unmodified | BSD-2-Clause, [`data/LICENSE-gpsd`](data/LICENSE-gpsd) |
| `data/gofree-merrimac-ais.nmea` | [Signal K server](https://github.com/SignalK/signalk-server) `samples/gofree-merrimac.log`, the `!AIVDM` lines only | Apache-2.0, [`data/LICENSE-signalk`](data/LICENSE-signalk) |
| `data/ublox-neo-m8p.ubx` | gpsd `test/daemon/ublox-neo-m8p.log`, unmodified | BSD-2-Clause, [`data/LICENSE-gpsd`](data/LICENSE-gpsd) |
| `data/tum-fendt820-field5.csv` | K. Götz, TUM, [*Agricultural Load Cycles*](https://doi.org/10.5281/zenodo.14619787): `Fendt 820/Cultivating (deep)/Field_5.csv`, 8210 rows, five of 26 columns | CC BY 4.0, [`data/LICENSE-tum`](data/LICENSE-tum) |

Recorded data from other equipment is welcome: see *Help wanted* in the
[main README](../README.md).
