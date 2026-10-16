#!/usr/bin/env python3
"""Writes tests/data/ubx_reference.txt: UBX messages of the gpsd regression
logs as pyubx2 decodes them, for tests/reference.rs to compare against.

For each log with UBX frames and each message type read by kinavis-ubx, the
first 40 messages: log name, occurrence of the type in the log (from 1),
message name,
then name=value fields in pyubx2's units (degrees, raw integers).

    python3 -I tools/reference.py path/to/gpsd/test/daemon

Requires pyubx2.
"""

import collections
import os
import sys

from pyubx2 import UBXReader, UBX_PROTOCOL

TYPES = {
    "NAV-PVT": ["iTOW", "fixType", "gnssFixOk", "diffSoln", "carrSoln", "numSV", "lon", "lat",
                "height", "hMSL", "hAcc", "vAcc", "velN", "velE", "velD", "gSpeed", "headMot",
                "pDOP", "validDate", "validTime"],
    "NAV-HPPOSLLH": ["iTOW", "lon", "lat", "height", "hMSL", "hAcc", "vAcc", "invalidLlh"],
    "NAV-RELPOSNED": ["version", "iTOW", "relPosN", "relPosE", "relPosD", "relPosLength",
                      "relPosHeading", "relPosHPN", "relPosHPE", "relPosHPD", "accN", "accE",
                      "accD", "relPosValid", "carrSoln", "relPosHeadingValid"],
    "NAV-SAT": ["iTOW", "numSvs"],
    "NAV-DOP": ["iTOW", "gDOP", "pDOP", "hDOP", "vDOP"],
    "NAV-STATUS": ["iTOW", "gpsFix", "gpsFixOk", "ttff", "msss"],
    "NAV-TIMELS": ["iTOW", "currLs", "validCurrLs"],
    "ESF-INS": ["iTOW", "xAccel", "yAccel", "zAccel", "zAngRate"],
    "ESF-MEAS": ["timeTag", "numMeas", "id"],
}
LIMIT = 40


def fields(message, name):
    values = []
    for field in TYPES[name]:
        values.append(f"{field}={getattr(message, field, '')}")
    if name == "NAV-SAT":
        for i in range(1, message.numSvs + 1):
            n = f"_{i:02d}"
            values.append("sv=" + ",".join(str(getattr(message, f + n)) for f in
                                          ("gnssId", "svId", "cno", "elev", "azim", "prRes", "svUsed")))
    if name == "ESF-MEAS":
        for i in range(1, message.numMeas + 1):
            values.append(f"meas={getattr(message, f'dataType_{i:02d}')},{getattr(message, f'dataField_{i:02d}')}")
    return values


def main(directory):
    with open("tests/data/ubx_reference.txt", "w") as out:
        out.write("# log occurrence message fields\n")
        for log in sorted(os.listdir(directory)):
            path = os.path.join(directory, log)
            data = open(path, "rb").read()
            if data.count(b"\xb5\x62") < 6:
                continue
            counts = collections.Counter()
            frame = 0
            with open(path, "rb") as stream:
                reader = UBXReader(stream, protfilter=UBX_PROTOCOL, quitonerror=0)
                for raw, message in reader:
                    if message is None:
                        continue
                    frame += 1
                    name = message.identity
                    if name not in TYPES or counts[name] >= LIMIT:
                        continue
                    counts[name] += 1
                    out.write(f"{log} {counts[name]} {name} " + " ".join(fields(message, name)) + "\n")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
