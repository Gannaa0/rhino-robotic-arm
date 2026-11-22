"""Sanity tests for the Rhino kinematics module.

Run:  python3 test_kinematics.py
Covers: known poses, FK->IK round-trips, reach limits, wrist warning,
track assist, consistent units (cm) and joint ranges.
"""

import math
import rhino_kinematics as K

passed = failed = 0


def check(name, cond):
    global passed, failed
    if cond:
        passed += 1
        print(f"  PASS  {name}")
    else:
        failed += 1
        print(f"  FAIL  {name}")


def approx(a, b, tol=1e-6):
    return abs(a - b) <= tol


print("== Known poses ==")
# Ruler pose: arm straight up, gripper level -> tip at (L4, 0, L1+L2+L3)
p = K.fk_from_joints(130, 90, 90, 90, 0)
check("ruler pose tip = (13.5, 0, 67.7)",
      approx(p["tip"][0], 13.5) and approx(p["tip"][1], 0.0)
      and approx(p["tip"][2], 67.7))

# Documented dashboard read-out pose: J2=90, J3=124.5, J4=107 -> Z ~ 67.7
p2 = K.fk_from_joints(130, 90.0, 124.5, 107.0, 0.0)
check("documented read-out pose Z ~= 67.7", abs(p2["tip"][2] - 67.7) < 0.2)
check("documented read-out pose XY ~= 0", math.hypot(p2["tip"][0], p2["tip"][1]) < 0.3)

# Track offset: tip X follows the track position
p3 = K.fk_from_joints(130, 90, 90, 90, 60)
check("track 60cm -> tip X = 73.5", approx(p3["tip"][0], 73.5))

print("== FK -> IK round-trips (world frame) ==")
poses = [
    (130, 90, 90, 90, 0),      # ruler
    (130, 60, 70, 90, 20),
    (160, 45, 120, 90, 100),   # yawed, elbow bent
    (100, 120, 60, 90, 0),
    (200, 30, 100, 90, 140),
]
for (j1, j2, j3, j4, tr) in poses:
    tip = K.fk_from_joints(j1, j2, j3, j4, tr)["tip"]
    ok, res = K.ik_to_joints(*tip, track=tr)
    if not ok:
        check(f"round-trip ({j1},{j2},{j3},{j4})", False)
        continue
    back = K.fk_from_joints(res["j1"], res["j2"], res["j3"], res["j4"],
                            res["track"])["tip"]
    err = math.dist(tip, back)
    check(f"round-trip J=({j1},{j2},{j3},{j4}) err={err:.2e}cm", err < 1e-6)

print("== Track assist ==")
# target beyond the track end: only reachable after sliding the track
ok, res = K.ik_to_joints(175, 0, 50, track=0)
check("track slides to reach far side target", ok and res["track"] > 100)

print("== Validation behaviour ==")
st, msg, sug = K.validate_target(13.5, 0, 67.7)
check("ruler target inside range", st == "ok")
st, msg, sug = K.validate_target(0, 300, 10)
check("far target out of bounds", st == "bad" and sug is not None)
if sug is not None:
    ok2, _ = K.ik_to_joints(*sug)
    check("suggested point is reachable", ok2)
st, msg, _ = K.validate_target(13.5, 0, 23.4)   # arm nearly folded flat
check("folded pose triggers wrist warning", st in ("warn", "bad"))

print("== Units & limits ==")
check("max reach = 44.5 cm", approx(K.MAX_REACH, 44.5))
check("link sum L1..L4 = 81.2 cm", approx(K.L1 + K.L2 + K.L3 + K.L4, 81.2))
check("gripper range 70..160", K.GRIP_OPEN == 70.0 and K.GRIP_CLOSED == 160.0)
cl = K.workspace_cloud(n=200)
check("workspace cloud generates points", len(cl) == 200)
ext = K.MAX_REACH + K.L4
check("cloud points inside track+reach volume",
      all(-ext <= pt[0] <= K.TRACK_MAX + ext and abs(pt[1]) <= ext
          and 0 <= pt[2] <= K.L1 + ext for pt in cl))

print(f"\n{passed} passed, {failed} failed")
raise SystemExit(1 if failed else 0)
