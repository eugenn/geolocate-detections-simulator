"""Reference vectors from the real akridia_v0_gld package, for the web simulator.

Run with the akridia conda env. Writes ref.json beside this file.
"""
import json
import math
import sys
from pathlib import Path

ROOT = Path("/Users/eugenn/Projects/softeq/akridia/softeq")
sys.path.insert(0, str(ROOT / "vision-perception-pipeline/geolocate-detections/implementation/geolocate-detections"))
sys.path.insert(0, str(ROOT / "deployment/platform-runtime-control-plane/implementation/messages"))

import torch  # noqa: E402

from akridia_v0_gld import tolerance as tol  # noqa: E402
from akridia_v0_gld.camera_model import CalibrationRecord, CameraModel  # noqa: E402
from akridia_v0_gld.sightline import cast_sightlines  # noqa: E402

torch.set_default_dtype(torch.float64)
D2R = math.pi / 180


def quat_from_ypr(yaw, pitch, roll):
    """Body FRD -> NED, ZYX (yaw, pitch, roll) in degrees, (w, x, y, z)."""
    cy, sy = math.cos(yaw * D2R / 2), math.sin(yaw * D2R / 2)
    cp, sp = math.cos(pitch * D2R / 2), math.sin(pitch * D2R / 2)
    cr, sr = math.cos(roll * D2R / 2), math.sin(roll * D2R / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


W, H, HFOV = 1920, 1080, 45.0
FX = (W / 2) / math.tan(HFOV * D2R / 2)
rec = CalibrationRecord(
    calibration_id="SIM", measured_conditions="synthetic", native_width_px=W, native_height_px=H,
    focal_x_px=FX, focal_y_px=FX, principal_x_px=W / 2, principal_y_px=H / 2,
    reference_hfov_deg=HFOV, distortion=(0.0, 0.0, 0.0, 0.0, 0.0), residual_px=0.3,
    lever_arm_body=(0.0, 0.0, 0.0))
cam = CameraModel(rec, torch.device("cpu"))
MOUNT = torch.eye(3)


def chain(u, v, pan, tilt, yaw, pitch, roll, pos, hfov=HFOV, lever=(0, 0, 0)):
    rays = cam.pixel_rays(torch.tensor([[u, v]], dtype=torch.float64), hfov).double()
    sl = cast_sightlines(rays, torch.tensor([pan * D2R]), torch.tensor([tilt * D2R]),
                         torch.tensor([quat_from_ypr(yaw, pitch, roll)]),
                         torch.tensor([pos], dtype=torch.float64),
                         torch.tensor(lever, dtype=torch.float64), MOUNT.double())
    o = sl.origin_world_ned[0].tolist()
    d = sl.direction_world_ned[0].tolist()
    # flat ground at D = 0
    t = -o[2] / d[2] if d[2] > 0 else None
    p = [o[i] + t * d[i] for i in range(3)] if t else None
    return {"ray_cam": rays[0].tolist(), "origin": o, "dir": d,
            "depression_deg": float(sl.depression_deg[0]), "ground": p,
            "slant": t}


cases = []
for (u, v, pan, tilt, yaw, pitch, roll) in [
    (960, 540, 0, -45, 0, 0, 0),
    (960, 540, 30, -45, 0, 0, 0),
    (1500, 900, 0, -45, 0, 0, 0),
    (960, 540, 0, -45, 40, 0, 0),
    (960, 540, 0, -45, 0, 10, 0),
    (960, 540, 0, -45, 0, 0, 10),
    (1200, 800, 70, -30, 120, 8, -12),
    (300, 1000, -110, -60, 250, -15, 20),
]:
    cases.append({"in": dict(u=u, v=v, pan=pan, tilt=tilt, yaw=yaw, pitch=pitch, roll=roll,
                             pos=[0, 0, -100]),
                  "out": chain(u, v, pan, tilt, yaw, pitch, roll, [0, 0, -100])})


def disp(base, pert):
    return [pert["ground"][i] - base["ground"][i] for i in range(2)]


# Error injection through the real chain at depression 45 and 25, h = 100 m.
inject = []
for dep in (45.0, 25.0):
    base = chain(960, 540, 0, -dep, 0, 0, 0, [0, 0, -100])
    slant = base["slant"]
    for name, kw in [("tilt", dict(tilt=-dep - 0.5)), ("pitch", dict(pitch=0.5)),
                     ("heading", dict(yaw=0.5)), ("roll", dict(roll=0.5))]:
        args = dict(u=960, v=540, pan=0, tilt=-dep, yaw=0, pitch=0, roll=0, pos=[0, 0, -100])
        args.update(kw)
        p = chain(**args)
        dn, de = disp(base, p)
        inject.append({"dep": dep, "source": name, "angle_deg": 0.5, "dn": dn, "de": de,
                       "measured": math.hypot(dn, de),
                       "spec_formula": float(tol.pointing_term(torch.tensor([slant]), 0.5)[0])})
    # vertical position error 2.5 m (aircraft lower than believed)
    p = chain(960, 540, 0, -dep, 0, 0, 0, [0, 0, -97.5])
    dn, de = disp(base, p)
    inject.append({"dep": dep, "source": "vertical", "dz_m": 2.5, "dn": dn, "de": de,
                   "measured": math.hypot(dn, de), "spec_formula": 0.0})

# Spec terms at the worked geometry, from tolerance.py itself.
s = tol.slant_range_m(100.0, 45.0).double().reshape(1)
dep = torch.tensor([45.0])
gsd = 100 * (HFOV * D2R / W) / math.sin(45 * D2R) ** 2
terms = {
    "slant": float(s[0]),
    "pointing_0p5deg": float(tol.pointing_term(s, 0.5)[0]),
    "pointing_1deg": float(tol.pointing_term(s, 1.0)[0]),
    "terrain_0p3m": float(tol.terrain_term(dep, 0.3)[0]),
    "premap_3m_2m": float(tol.premap_georef_term(s, 3.0, 2.0)[0]),
    "timing_50ms_90dps": float(tol.timing_term(s, 50.0, 90.0)[0]),
    "timing_2ms_90dps": float(tol.timing_term(s, 2.0, 90.0)[0]),
    "camera_0p3px": float(tol.camera_model_term(s, 0.3, FX)[0]),
    "box_5px": float(tol.box_lower_edge_term(torch.tensor([gsd]), 5.0)[0]),
    "gsd_along_m": gsd,
}
et = tol.ErrorTerms(*(torch.tensor([terms[k]]) for k in
                      ("pointing_0p5deg", "terrain_0p3m", "premap_3m_2m", "timing_2ms_90dps",
                       "camera_0p3px", "box_5px")))
t, dom = tol.compose(et)
terms["composed"] = float(t[0])
terms["dominant"] = tol.dominant_term_for(int(dom[0])).name

out = {"generated_by": "akridia_v0_gld via refgen.py", "fx": FX, "W": W, "H": H, "hfov": HFOV,
       "cases": cases, "inject": inject, "terms": terms,
       "torch": torch.__version__}
Path(__file__).with_name("ref.json").write_text(json.dumps(out, indent=1))
for r in inject:
    print(f"{r['dep']:4.0f} {r['source']:8s} measured {r['measured']:.3f}  spec {r['spec_formula']:.3f}")
print(json.dumps(terms, indent=1))
