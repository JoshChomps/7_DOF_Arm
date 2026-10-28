"""Set up and check the MKS SERVO42D/57D drivers over CAN from one table.

    python -m deskarm.drivers --channel COM5                 # read every driver back, compare with the table
    python -m deskarm.drivers --channel COM5 --write         # write what differs, then read back again
    python -m deskarm.drivers --channel COM5 --assign J3     # ONE driver on the bus, still id 1: give it J3's id
    python -m deskarm.drivers --channel COM5 --calibrate J3  # encoder calibration; the motor must be unloaded
    python -m deskarm.drivers --channel COM5 --jog J3        # one motor turn the positive way: finds motor_sign
    python -m deskarm.drivers --sim                          # any of the above against simulated drivers

Every driver ships as CAN id 1, so the first job on each is ``--assign`` with only that driver on the bus.
Read-back is the manual's "read configuration parameter" (V1.0.9 §5.1.10: send ``00 <code>``, the reply
is ``<code> <value>`` in the same format as the write, or ``FF FF`` if that setting can't be read).
Not in the table, on purpose:

* CanRate: if the tool can talk to a driver, it is at the bridge's 500k already.
* Dir (86H): only for the pulse interface (§5.2.7). Over CAN the sign is ``motor_sign`` in arm.toml.
* En (85H): in a bus mode the En pin is ignored and F3 enables the motor (§9.2.2).
* HoldMa (9BH): SR_vFOC adapts the current up to Ma and ignores it.
* Position-error protection (9DH): off by default; ``CanBackend`` watches the stall flag (3EH) instead.

Written from the manual and **not yet run against a driver**.
"""
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

from . import mks

READ = 0x00              # read a configuration parameter
VERSION = 0x40           # calibrated flag, board type, firmware version
CALIBRATE = 0x80
CAN_ID = 0x8B
BOARDS = {1: "S42D_485", 2: "S42D_CAN", 3: "S57D_485", 4: "S57D_CAN",
          5: "S28D_RS485", 6: "S28D_CAN", 7: "S35D_RS485", 8: "S35D_CAN"}
MAX_MA = {"S42D_CAN": 3000, "S57D_CAN": 5200}            # manual §5.2.3


@dataclass(frozen=True)
class Setting:
    name: str
    code: int
    size: int            # bytes, big-endian
    why: str


SETTINGS = (
    Setting("mode", 0x82, 1, "5 = SR_vFOC: bus control; the current rises to Ma when the load needs it"),
    Setting("current_ma", 0x83, 2, "Ma = the motor's rated current until the peak-or-RMS bench test"),
    Setting("microsteps", 0x84, 1, "16: the F5 speed field is rpm only at 16/32/64 (manual sec. 9.1)"),
    Setting("interpolation", 0x89, 1, "1 = on (the default)"),
    Setting("overcurrent_stop", 0x88, 1, "0 = off (the default): tripping it releases the shaft and the arm drops"),
    Setting("respond", 0x8C, 2, "0x0101 = reply, and report when a move ends (the default); CanBackend reads both"),
)


def wanted(arm, j: int) -> dict[str, int]:
    """The table's value of every setting for joint index ``j``."""
    return {"mode": 5, "current_ma": round(arm.joints[j].rated_current * 1000), "microsteps": 16,
            "interpolation": 1, "overcurrent_stop": 0, "respond": 0x0101}


def request(bus, arb_data: tuple[int, bytes], answer: int | None = None, timeout: float = 0.5) -> bytes | None:
    """Send one ``mks.frame`` and wait for that driver's reply with code ``answer`` (default: the frame's)."""
    can_id, data = arb_data
    bus.send(mks.message(arb_data))
    answer = data[0] if answer is None else answer
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(0.01)
        r = msg and msg.arbitration_id == can_id and mks.reply(can_id, msg.data)
        if r and r[0] == answer:
            return r[1]
    return None


def read_setting(bus, can_id: int, s: Setting) -> int | None:
    p = request(bus, mks.frame(can_id, READ, bytes([s.code])), answer=s.code)
    return None if p is None or p == b"\xff\xff" or len(p) != s.size else int.from_bytes(p, "big")


def write_setting(bus, can_id: int, s: Setting, value: int) -> bool:
    p = request(bus, mks.frame(can_id, s.code, value.to_bytes(s.size, "big")))
    return p is not None and p[:1] in (b"\x01", b"\x02")        # 83H: 1 = saved, 2 = set but not saved


def version(bus, can_id: int) -> tuple[bool, str, str] | None:
    """(calibrated, board type, firmware version) from 40H, or None if the driver doesn't answer."""
    p = request(bus, mks.frame(can_id, VERSION))
    if p is None or len(p) < 3:
        return None
    return bool(p[0] >> 4), BOARDS.get(p[0] & 0x0F, f"unknown ({p[0] & 0x0F})"), p[1:].hex()


def check(arm, bus, ids=None, write=False, out=print) -> bool:
    """Compare every driver with the table (and write what differs if ``write``). True if all match."""
    ids, ok = list(ids or range(1, arm.n + 1)), True
    for j, cid in enumerate(ids):
        joint = arm.joints[j]
        v = version(bus, cid)
        if v is None:
            out(f"{joint.name} (id {cid}): no reply. Check power, CAN wiring, the id and 500k")
            ok = False
            continue
        cal, board, firm = v
        out(f"{joint.name} (id {cid}): {board}, firmware {firm}, {'calibrated' if cal else 'NOT CALIBRATED'}"
            + ("" if board == joint.driver else f"  <- arm.toml says {joint.driver}"))
        ok &= cal and board == joint.driver
        want = wanted(arm, j)
        if want["current_ma"] > MAX_MA.get(board, 3000):
            out(f"  current_ma {want['current_ma']} is above this board's maximum: not written")
            ok = False
        for s in SETTINGS:
            have = read_setting(bus, cid, s)
            if have == want[s.name]:
                continue
            fix = write and not (s.name == "current_ma" and want[s.name] > MAX_MA.get(board, 3000))
            done = fix and write_setting(bus, cid, s, want[s.name]) and read_setting(bus, cid, s) == want[s.name]
            out(f"  {s.name}: {'unreadable' if have is None else have} -> {want[s.name]}"
                f"{'  written' if done else '  WRITE FAILED' if fix else ''}   ({s.why})")
            ok &= bool(done)
    return ok


def assign(bus, new_id: int, out=print) -> bool:
    """Give the only driver on the bus (still at the factory id 1) the id ``new_id``."""
    if version(bus, 1) is None:
        out("no driver answers at id 1: is exactly one, freshly unboxed (or reset) driver on the bus?")
        return False
    p = request(bus, mks.frame(1, CAN_ID, new_id.to_bytes(2, "big")))
    done = p == b"\x01" and version(bus, new_id) is not None
    out(f"id 1 -> {new_id}: {'done' if done else 'FAILED'}")
    return done


def calibrate(bus, can_id: int, out=print, timeout: float = 60.0) -> bool:
    """Encoder calibration (80H). The motor must be unloaded: off the gearbox, or the belt off."""
    bus.send(mks.message(mks.frame(can_id, CALIBRATE, b"\x00")))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(0.1)
        r = msg and msg.arbitration_id == can_id and mks.reply(can_id, msg.data)
        if r and r[0] == CALIBRATE and r[1][:1] in (b"\x01", b"\x02"):
            out(f"id {can_id}: calibration {'done' if r[1][0] == 1 else 'FAILED'}")
            return r[1][0] == 1
    out(f"id {can_id}: no calibration result within {timeout:.0f} s")
    return False


def jog(arm, bus, j: int, can_id: int, turns: float = 1.0, rpm: int = 30, out=print) -> bool:
    """Turn joint ``j``'s motor ``turns`` motor turns toward positive counts, so you can see the sign.

    The joint moves 360° x turns / ratio (one turn is 7.5° on J2-J4). The motor stays enabled afterwards.
    """
    p = request(bus, mks.frame(can_id, mks.AXIS))
    if p is None or request(bus, mks.frame(can_id, mks.ENABLE, b"\x01")) != b"\x01":
        out(f"id {can_id}: no reply or not enabled. Is it in SR_vFOC (run the check first)?")
        return False
    goal = int.from_bytes(p, "big", signed=True) + round(turns * mks.COUNTS)
    started = request(bus, mks.move_frame(can_id, rpm, 0, goal)) == bytes([mks.MOVE_STARTED])
    joint = arm.joints[j]
    out(f"{joint.name}: {turns:g} motor turn(s) at {rpm} rpm = {360 * turns / joint.ratio:.1f} deg at the joint"
        + ("" if started else "  (the driver did not confirm the move)") + ".\n"
        "If it turned the positive way (right-hand rule about the joint's axis: J1/J3/J5/J7 along +Z and"
        " J2/J4/J6 along +Y at the zero pose, docs/16 sec. 4), leave motor_sign = 1 in arm.toml; otherwise set -1.")
    return started


def main(argv=None) -> int:
    from .kinematics import load_arm

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--channel", help="the bridge's serial port (COM5, /dev/ttyACM0)")
    ap.add_argument("--sim", action="store_true", help="simulated drivers instead of a bridge")
    ap.add_argument("--write", action="store_true", help="write the settings that differ from the table")
    ap.add_argument("--assign", metavar="JOINT", help="give the one driver at id 1 this joint's id")
    ap.add_argument("--calibrate", metavar="JOINT", help="calibrate this joint's encoder (motor unloaded!)")
    ap.add_argument("--jog", metavar="JOINT", help="one motor turn the positive way, to find motor_sign")
    a = ap.parse_args(argv)
    arm = load_arm()
    names = [jt.name for jt in arm.joints]
    if not a.sim and not a.channel:
        ap.error("give --channel (or --sim)")
    if a.sim:
        boards = {k + 1: 4 for k, jt in enumerate(arm.joints) if jt.driver == "S57D_CAN"}
        bus = mks.SimBus(range(1, 2) if a.assign else range(1, arm.n + 1), boards=boards)
    else:
        from .bridge import open_bus
        bus = open_bus(a.channel)
    try:
        for flag in (a.assign, a.calibrate, a.jog):
            if flag and flag not in names:
                ap.error(f"unknown joint {flag}: one of {', '.join(names)}")
        if a.assign:
            return 0 if assign(bus, names.index(a.assign) + 1) else 1
        if a.calibrate:
            return 0 if calibrate(bus, names.index(a.calibrate) + 1) else 1
        if a.jog:
            j = names.index(a.jog)
            return 0 if jog(arm, bus, j, j + 1) else 1
        return 0 if check(arm, bus, write=a.write) else 1
    finally:
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
