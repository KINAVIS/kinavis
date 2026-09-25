# KINAVIS demonstrations

Four short programs, each run on data recorded from real equipment. Nothing
here is simulated except where it says so.

```sh
cargo run -p kinavis-examples --example receivers
cargo run -p kinavis-examples --example gnss_jump
cargo run -p kinavis-examples --example traffic
cargo run -p kinavis-examples --example course_to_steer
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

## Data

| File | Source | Licence |
|---|---|---|
| `data/receivers.nmea` | [gpsd](https://gitlab.com/gpsd/gpsd) regression logs, `test/daemon/`, one line from each of seven | BSD-2-Clause, [`data/LICENSE-gpsd`](data/LICENSE-gpsd) |
| `data/bundg_zeus_9.nmea` | gpsd `test/daemon/bundg_zeus_9.log`, unmodified | BSD-2-Clause, [`data/LICENSE-gpsd`](data/LICENSE-gpsd) |
| `data/gofree-merrimac-ais.nmea` | [Signal K server](https://github.com/SignalK/signalk-server) `samples/gofree-merrimac.log`, the `!AIVDM` lines only | Apache-2.0, [`data/LICENSE-signalk`](data/LICENSE-signalk) |

Recorded data from other equipment is welcome: see *Help wanted* in the
[main README](../README.md).
