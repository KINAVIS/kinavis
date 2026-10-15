#!/usr/bin/env python3
"""Writes tests/data/mavlink_reference.txt from pymavlink.

For each message of kinavis-mavlink, random field values (floats chosen
exact in binary so that both sides print them alike), the MAVLink 2 frame
pymavlink encodes for them, and the struct as Rust's Debug prints it; then
frames signed with a known key. Only the values are kept.
Columns: kind, hex frame, expected Debug text (`unsigned`, `v1`) or link
and timestamp (`signed`); `v1` frames are MAVLink 1.

    python3 -I tools/mavlink_reference.py

Requires pymavlink (LGPLv3; used here, never shipped).
"""

import random

from pymavlink.dialects.v10 import ardupilotmega as mavlink1
from pymavlink.dialects.v20 import ardupilotmega as mavlink

rng = random.Random(20261016)
KEY = bytes(range(32))
NAMES = ["HEARTBEAT", "GPS_RAW_INT", "ATTITUDE", "LOCAL_POSITION_NED",
         "GLOBAL_POSITION_INT", "MISSION_ITEM_INT",
         "SET_POSITION_TARGET_GLOBAL_INT", "GPS_RTCM_DATA"]


class Sink:
    def write(self, data):
        pass


def value(kind, zero):
    if zero:
        return 0.0 if kind in ("float", "double") else 0
    if kind in ("float", "double"):
        return rng.randint(-4000, 4000) / 8.0
    bits = {"uint8_t": (0, 255), "int8_t": (-128, 127), "uint16_t": (0, 65535),
            "int16_t": (-32768, 32767), "uint32_t": (0, 2**32 - 1),
            "int32_t": (-2**31, 2**31 - 1), "uint64_t": (0, 2**64 - 1),
            "int64_t": (-2**63, 2**63 - 1)}
    low, high = bits[kind.replace("_mavlink_version", "")]
    return rng.randint(low, high)


def debug(name, message, cls):
    rust = "".join(part.capitalize() for part in name.split("_"))
    parts = []
    for field, kind, length in zip(cls.fieldnames, cls.fieldtypes, cls.array_lengths):
        v = getattr(message, field)
        if length:
            items = list(v) if not isinstance(v, (bytes, str)) else list(v.encode() if isinstance(v, str) else v)
            items += [0] * (length - len(items))
            text = "[" + ", ".join(fmt(kind, x) for x in items) + "]"
        else:
            text = fmt(kind, v)
        parts.append(f"{'mav_type' if field == 'type' else field}: {text}")
    return rust + " { " + ", ".join(parts) + " }"


def fmt(kind, v):
    if kind in ("float", "double"):
        return repr(float(v))
    return str(int(v))


print(__doc__.splitlines()[0])
with open("tests/data/mavlink_reference.txt", "w") as out:
    out.write("# kind frame expected\n")
    mav = mavlink.MAVLink(Sink(), srcSystem=1, srcComponent=1)
    for round_ in range(40):
        for name in NAMES:
            cls = getattr(mavlink, "MAVLink_" + name.lower() + "_message")
            # Every fifth message of a kind ends in zeros, to be truncated.
            zero_tail = round_ % 5 == 0
            args = []
            for index, (field, kind, length) in enumerate(zip(cls.fieldnames, cls.fieldtypes, cls.array_lengths)):
                zero = zero_tail and index >= len(cls.fieldnames) // 2
                if length:
                    args.append([value(kind, zero) for _ in range(length)])
                else:
                    args.append(value(kind, zero))
            message = cls(*args)
            mav.seq = rng.randint(0, 255)
            frame = message.pack(mav)
            out.write(f"unsigned {frame.hex()} {debug(name, message, cls)}\n")
    signer = mavlink.MAVLink(Sink(), srcSystem=7, srcComponent=190)
    signer.signing.secret_key = KEY
    signer.signing.link_id = 3
    signer.signing.sign_outgoing = True
    for index in range(20):
        signer.signing.timestamp = 600_000_000_000 + index * 1234
        message = mavlink.MAVLink_attitude_message(index, 0.5, -0.25, 1.0, 0.0, 0.125, -2.0)
        frame = message.pack(signer)
        out.write(f"signed {frame.hex()} 3 {600_000_000_000 + index * 1234}\n")
    old = mavlink1.MAVLink(Sink(), srcSystem=2, srcComponent=1)
    for index in range(10):
        message = mavlink1.MAVLink_attitude_message(index * 10, 0.25, 0.5, -1.5, 0.0, 0.0, 0.75)
        out.write(f"v1 {message.pack(old).hex()} {debug('ATTITUDE', message, mavlink.MAVLink_attitude_message)}\n")
