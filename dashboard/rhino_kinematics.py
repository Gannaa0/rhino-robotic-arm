"""
Rhino robotic arm - shared kinematics (dashboard side).

Reconstructed from the project's graduation documentation:
geometric FK/IK for the 5-DOF arm (track + J1 base, J2 shoulder,
J3 elbow, J4 wrist) with the gripper kept level.

Frames and conventions (verified against the documented dashboard
read-out J2=90, J3=124.5, J4=107 -> tip Z = 67.7 cm):
  - World frame: X = side, along the linear track; Y = forward; Z = up.
    The arm's base sits at (track, 0, 0).
  - Cartesian (XYZ) targets are world-frame. The IK first tries the
    current track position; if the target is out of reach it slides the
    track (0..140 cm) toward the target and solves again, mirroring the
    documented behaviour where L moves the track and XYZ moves the wrist.
  - J1 slider 0..260  <=> base yaw -130..+130 deg (theta1 = atan2(Y, X)
    in the arm-local frame, per the documentation's IK geometry).
  - J2 slider = elevation of link L2 from horizontal (90 = vertical,
    the "ruler" pose).
  - J3 slider = 90 + signed elbow bend relative to L2.
  - J4 slider = 90 + absolute wrist elevation (90 = gripper level).
  - Gripper G: 70 = open .. 160 = closed.

All lengths in centimeters, all angles in degrees unless noted.
"""

import math

# ---- Mechanical constants (from the documentation) ----
L1 = 23.2          # base height (cm)
L2 = 22.0          # shoulder link (cm)
L3 = 22.5          # elbow link (cm)
L4 = 13.5          # wrist link (cm)
TRACK_MIN, TRACK_MAX = 0.0, 140.0     # linear track travel (cm)
BASE_MIN, BASE_MAX = -130.0, 130.0    # J1 base yaw (deg)
MAX_REACH = L2 + L3                   # 44.5 cm wrist reach (documented)
MIN_REACH = abs(L2 - L3)              # 0.5 cm

# Joint slider ranges (from the dashboard specification)
J1_MIN, J1_MAX = 0.0, 260.0
J2_MIN, J2_MAX = 0.0, 200.0
J3_MIN, J3_MAX = 0.0, 250.0
J4_MIN, J4_MAX = 0.0, 200.0
GRIP_OPEN, GRIP_CLOSED = 70.0, 160.0

# Wrist mechanical bend limit (deg the wrist joint can compensate).
# Beyond this the pose is reachable but the gripper cannot stay level.
WRIST_REL_LIMIT = 100.0


def _d(x):
    return math.radians(x)


def _g(x):
    return math.degrees(x)


def fk_from_joints(j1, j2, j3, j4, track):
    """Forward kinematics: joint slider values -> world-frame 3D points.

    Returns a dict with the 3D coordinates (cm) of every joint:
    base, shoulder, elbow, wrist, tip.
    """
    yaw = _d(j1 - 130.0)          # slider 0..260 <=> -130..+130 deg
    phi2 = _d(j2)                 # L2 elevation from horizontal
    phi3 = phi2 + _d(j3 - 90.0)   # L3 absolute elevation
    phi4 = _d(j4 - 90.0)          # L4 (gripper) absolute elevation

    ux, uy = math.cos(yaw), math.sin(yaw)

    base = (track, 0.0, 0.0)
    shoulder = (track, 0.0, L1)

    r2 = L2 * math.cos(phi2)
    elbow = (track + r2 * ux, r2 * uy, L1 + L2 * math.sin(phi2))

    r3 = L3 * math.cos(phi3)
    wrist = (elbow[0] + r3 * ux, elbow[1] + r3 * uy, elbow[2] + L3 * math.sin(phi3))

    r4 = L4 * math.cos(phi4)
    tip = (wrist[0] + r4 * ux, wrist[1] + r4 * uy, wrist[2] + L4 * math.sin(phi4))

    return {
        "base": base,
        "shoulder": shoulder,
        "elbow": elbow,
        "wrist": wrist,
        "tip": tip,
        "yaw_deg": j1 - 130.0,
        "wrist_level": abs(j4 - 90.0) < 1e-9,
    }


def _solve_planar(r_w, s, elbow_down=True):
    """Planar 2R solve (law of cosines) for wrist centre (r_w, s).

    Returns (phi2_deg, theta3_deg): L2 elevation and signed elbow bend,
    or None when unreachable.
    """
    dist = math.hypot(r_w, s)
    if dist > MAX_REACH or dist < MIN_REACH:
        return None
    d = (r_w * r_w + s * s - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)
    d = max(-1.0, min(1.0, d))
    bend = math.sqrt(max(0.0, 1.0 - d * d))
    theta3 = math.atan2(-bend, d) if elbow_down else math.atan2(bend, d)
    phi2 = math.atan2(s, r_w) - math.atan2(L3 * math.sin(theta3),
                                          L2 + L3 * math.cos(theta3))
    return _g(phi2), _g(theta3)


def ik_to_joints(x, y, z, track=0.0, elbow_down=True):
    """Inverse kinematics (geometric method), world-frame tip target.

    Keeps the gripper level. Slides the track toward the target when the
    current track position cannot reach it.

    Returns (ok, result):
      ok=True  -> result = dict(j1..j4, track, wrist_level_ok, note)
      ok=False -> result = dict(reason, suggested=(x, y, z))
    """
    s = z - L1
    branches = (True, False) if elbow_down else (False, True)
    tried = []
    for t in (track, max(TRACK_MIN, min(TRACK_MAX, x))):
        if any(abs(t - u) < 1e-9 for u in tried):
            continue
        tried.append(t)
        lx = x - t
        r = math.hypot(lx, y)
        r_w = r - L4       # level gripper: wrist link lies horizontal
        for branch in branches:          # try both elbow configurations
            sol = _solve_planar(r_w, s, branch)
            if sol is None:
                break
            phi2, theta3 = sol
            j1 = _g(math.atan2(y, lx)) + 130.0
            j2 = phi2
            j3 = 90.0 + theta3
            j4 = 90.0
            if not (J1_MIN <= j1 <= J1_MAX):
                break
            if not (J2_MIN <= j2 <= J2_MAX) or not (J3_MIN <= j3 <= J3_MAX):
                continue
            # relative bend the wrist must compensate to stay level
            needed_rel = abs(phi2 + theta3)
            wrist_level_ok = needed_rel <= WRIST_REL_LIMIT
            return True, {
                "j1": j1, "j2": j2, "j3": j3, "j4": j4, "track": t,
                "wrist_level_ok": wrist_level_ok,
                "note": "" if wrist_level_ok else "reachable, but gripper NOT horizontal",
            }

    # nearest reachable suggestion along the same direction
    t = max(TRACK_MIN, min(TRACK_MAX, x))
    lx = x - t
    r = math.hypot(lx, y)
    r_w = r - L4
    dist = math.hypot(r_w, s) or 1e-9
    scale = (MAX_REACH - 0.5) / dist if dist > MAX_REACH else (MIN_REACH + 0.5) / dist
    sug_r_w = r_w * scale
    sug_s = s * scale
    ratio = (sug_r_w + L4) / r if r else 1.0
    suggested = (t + lx * ratio, y * ratio, sug_s + L1)
    return False, {"reason": "out of bounds", "suggested": suggested}


def validate_target(x, y, z, track=0.0):
    """Dashboard live validation for the Cartesian control panel.

    Returns (status, message, suggested):
      status: "ok" | "warn" | "bad"
    """
    ok, res = ik_to_joints(x, y, z, track)
    if not ok:
        return "bad", "out of bounds", res.get("suggested")
    if not res["wrist_level_ok"]:
        return "warn", "reachable, but gripper NOT horizontal", None
    return "ok", "inside range", None


def workspace_cloud(n=4000, seed=42):
    """Reachable-workspace point cloud (tip positions, gripper level).

    Used by the dashboard's 'Cloud' toggle and the Plotly full-workspace view.
    """
    import random
    rnd = random.Random(seed)
    pts = []
    while len(pts) < n:
        j2 = rnd.uniform(J2_MIN, J2_MAX)
        j3 = rnd.uniform(J3_MIN, J3_MAX)
        j1 = rnd.uniform(J1_MIN, J1_MAX)
        track = rnd.uniform(TRACK_MIN, TRACK_MAX)
        if abs(j2 + (j3 - 90.0)) > WRIST_REL_LIMIT:
            continue
        pts.append(fk_from_joints(j1, j2, j3, 90.0, track)["tip"])
    return pts
