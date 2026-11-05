/* UI strings for the Geolocate Detections Lab. English only.
 * Values may contain {name} placeholders. Arrays are returned as they are. */
window.I18N = {
  en: {
    "lede": "A simulator of the chain “box → ray → ground → tolerance” (M-V0-820…823). The geometry is a port of <code>akridia_v0_gld</code>, checked against it on the “Code check” tab. It shows how the pipeline works and tests the review findings: where the spec's formulas depart from the real geometry and what the fix gives.",
    "nav.aria": "Sections",
    "tab.pipeline": "How it works", "tab.budget": "Error budget", "tab.fusion": "Multiple looks", "tab.findings": "Findings", "tab.ref": "Code check",
    "lang.aria": "Language",

    "u.m": "m", "u.deg": "°", "u.dps": "°/s", "u.ms": "ms", "u.mps": "m/s", "u.px": "px",

    "grp.POINTING": "Pointing", "grp.TERRAIN": "Terrain", "grp.PREMAP_GEOREF": "Pre-map and aircraft position",
    "grp.TIMING": "Timing", "grp.CAMERA_MODEL": "Camera model", "grp.BOX_LOWER_EDGE": "Box lower edge",

    "pat.one": "1 look", "pat.same": "4 looks, same heading", "pat.opp": "2 opposite headings",
    "pat.four": "4 headings, 90° apart", "pat.orbit": "orbit, 8 headings",

    /* pipeline tab */
    "geo.title": "Frame geometry", "geo.air": "Aircraft", "geo.gim": "Gimbal and camera", "geo.ter": "Terrain",
    "geo.h": "Height above ground", 
    "geo.pan": "Gimbal pan", "geo.tilt": "Gimbal tilt (− is down)", 
    "geo.slope": "Slope falling away from the camera", 
    "fig.frame": "Frame 1920 × 1080 (EO)", "fig.frame.aria": "Camera frame",
    "fig.frame.cap": "Click the frame to place the box. The ray goes through the bottom-centre of the box (§2.4): u = (x_min + x_max) / 2, v = y_max.",
    "fig.side": "Side view, along the line of sight", "fig.side.aria": "Side view",
    "fig.top": "Top view: frame footprint on the ground", "fig.top.aria": "Top view",
    "fig.top.cap": "North is up. The outline is the frame edges projected onto the surface; rays above the horizon are clipped at 1500 m. The dot is the intersection of the ray from the bottom-centre of the box.",
    "side.h": "h = {h} m", "side.S": "S = {s} m", "side.R": "R = {r} m",
    "side.cap.hit": "The lever that turns an angular error into metres along the look direction: S / sin(depression) = {a} m per radian; the spec uses S = {s} m.",
    "side.cap.none": "The ray does not meet the surface: the pipeline emits a proposal with no position (NO_INTERSECTION).",
    "top.uav": "UAV", "top.m": "{x} m",
    "flow.s1": "Detector box", "flow.s2": "Camera model", "flow.s3": "Ray in the world", "flow.s4": "Terrain intersection",
    "flow.s5": "Tolerance §4.5", "flow.s6": "Covariance",
    "flow.id5": "M-V0-822 · spec", "flow.id6": "M-V0-822 · model",

    
    "flow.depression": "depression", "flow.azimuth": "azimuth",
    "flow.slant": "slant range", "flow.incidence": "incidence", "flow.refusal": "refusal", "flow.nointersect": "NO_INTERSECTION",

    "flow.sigAlong": "σ along", "flow.sigAcross": "σ across",

    /* budget tab */
    "bud.sources": "Error sources (1σ)", "bud.more": "Other sources: survey, timing, calibration",
    "bud.heading": "Heading", "bud.att": "Roll and pitch (each)", "bud.bore": "Boresight (each axis)",
    "bud.vert": "Aircraft height", "bud.gnss": "Aircraft GNSS, horizontal (σh, as on W-008)", "bud.survey": "Survey georeferencing, horizontal",
    "bud.terrain": "Terrain height", "bud.dt": "Frame-to-pose misalignment", "bud.rate": "Angular rate at that moment", "bud.speed": "Aircraft speed",
    "bud.cam": "Calibration residual", "bud.box": "Box lower edge", "bud.geometry": "Geometry",
    "bud.dep": "Look angle (frame centre)", "bud.hgt": "Height above ground",
    "bud.compare": "Tolerance models compared on the current geometry",

    "bud.nohit": "The ray does not meet the surface. Lower the camera or raise the look angle.",
    "tile.spec": "Spec tolerance (“95%”)", "tile.spec.s": "covers {x} of errors",
    "tile.model": "r95 from covariance", "tile.model.s": "covers {x} of errors",
    "tile.real": "Real 95% (Monte Carlo)", "tile.real.s": "{n} runs of the chain",

    "mc.low": "The spec tolerance is {r} times too small: the “95th percentile” label on W-001 does not match what it covers. The covariance r95 lands on 95% to within Monte Carlo noise.",
    "mc.ok": "With these sources the spec tolerance covers {x}. Try a larger height σ or a smaller look angle.",
    "sc.title": "Error scatter: Monte Carlo through the full chain", "sc.aria": "Error scatter",
    "sc.legend.spec": "spec tolerance", "sc.legend.r95": "r95 from covariance", "sc.legend.ell": "95% ellipse", "sc.look": "look direction",
    "terms.title": "Terms: spec vs real geometry",
    "terms.group": "Group", "terms.spec": "Spec, m", "terms.real": "Real, m", "terms.ratio": "Real / spec", "terms.notinspec": "not in spec", "terms.total": "Total",
    "terms.note": "“Spec” is the formulas of tolerance.py (§4.5): pointing angle × slant range, Δh / tan(look angle), a quadrature sum with no factor. “Real” is the contribution of the same group of sources to the covariance obtained by numerically differentiating the full chain. The spec's POINTING angle is taken as the RSS of all angular σ, which is the most favourable reading for the spec.",

    /* fusion tab */
    "fu.title": "Experiment", "fu.beta": "Share of bias in the variance", "fu.dep": "Look angle", 
    "fu.scheme": "Scheme",
    "fu.note": "Sources come from the Error budget tab. Bias (heading, roll and pitch, boresight, height, GNSS, survey, terrain) is the same for every look of one flight; box, calibration and timing noise is new on each look. Fusion is a weighted least squares over the look covariances.",
    "fu.bars": "95% radius by observation scheme", "fu.bars.aria": "95% radius by scheme",
    "fu.bars.cap": "Rust bars are what §1.2 allows (one best observation, or averaging from one heading). Blue bars fuse observations from different headings. The floor is set by what all looks share: the aircraft GNSS error (a survey shift matters only on a slope).",
    "fu.one": "1 look", "fu.10m": "10 m",

    /* findings tab */
    "fnd.title": "Findings and fixes",
    "fnd.intro": "Each finding is checked by a calculation on this page. “Reference” numbers come from running the <code>akridia_v0_gld</code> package itself (conda environment <code>akridia</code>, torch 2.5.1); the rest is computed here with the same geometry. The sliders inside a card change only that card.",
    "sev.crit": "critical", "sev.warn": "important", "sev.info": "note",
    "fnd.what": "What is wrong", "fnd.fix": "How to fix it",

    "F1.title": "Angular errors along the look direction cost S / sin(depression), not S",
    "F1.where": ["§4.5 rows 1, 4, 5", "tolerance.py:213, 295, 331", "deviation 16"],
    "F1.problem": "POINTING, TIMING and CAMERA_MODEL are computed as angle × slant range. A tilt or pitch error moves the intersection along the ground by more than that: the lever is S / sin(θ) = h / sin²(θ). The box-lower-edge term already includes this factor, so the six terms are inconsistent with each other.",
    "F1.fix": "Compute each angular source's contribution through the chain's Jacobian: in the look plane S / sin(θ), heading R across, roll h across. In code this is one function that both the ray and the tolerance call.",
    "F1.lbl": "Tilt 0.5°, h = 100 m, akridia_v0_gld run", "F1.c1": "spec S·δ", "F1.c2": "reference", "F1.c3": "S·δ / sin θ", "F1.c4": "underestimate", "F1.btn": "Open budget at 25°",

    "F2.title": "No term for aircraft vertical error",
    "F2.where": ["§4.5", "tolerance.py:270", "W-mavlink-fcs-bridge-008.sigma_v_m is not read"],
    "F2.problem": "An aircraft height error moves the point by Δz / tan(θ) along the look direction: 1:1 at 45°, twice that at 25°. In the nominal budget this is the largest term, and the spec has none. The sigma_v_m field arrives on the wire but is never read.",
    "F2.fix": "Add an “aircraft height” source (sigma_v_m) with lever 1 / tan(incidence). Remove the height bias on a pad of known height before take-off: σ drops from about 3 m to about 1 m, and r95 at 45° from 7.3 to 5.3 m.",
    "F2.lbl": "Aircraft height is off by 2.5 m, akridia_v0_gld run", "F2.c1": "spec", "F2.c2": "reference, m", "F2.c3": "Δz / tan θ", "F2.btn": "Compare height σ of 3 m and 1 m",

    

    

    "F4.title": "“95th percentile” without a factor: the tolerance covers about half of the errors",
    "F4.where": ["§7 W-001/002/003 position_tolerance_m", "tolerance.compose", "deviation 8"],
    "F4.problem": "Terms of unclear meaning (1σ or 95%) are summed in quadrature and the result is labelled a 95th percentile. On the Isaac bench the coverage was 47–55% with 1σ sizes. The Monte Carlo below repeats that with the current settings of the Error budget tab.",
    "F4.fix": "Define a term size as 1σ per source, compute the covariance, and report the radius of the circle holding 95% (2.4477σ for a circle, exact for an ellipse). Coverage becomes a testable property: at least 27 of 30 markers inside.",
    "F4.lbl": "45°, 100 m, sources from the Error budget tab, 2000 runs", "F4.spec": "spec tolerance", "F4.model": "covariance r95", "F4.btn": "Open Monte Carlo",

    "F5.title": "“One position, best observation, never averaged” loses the main gain",
    "F5.where": ["§1.2", "§1.5 (Recommended)", "report_path.py, deviation 9"],
    "F5.problem": "Biases dominate: heading, height, boresight. Averaging looks from one heading does not cancel them, while looks from opposite headings give them opposite signs. The spec itself relies on this in the two-heading gate (§8), yet reports a single observation.",
    "F5.fix": "For static entities use a covariance-weighted least squares with Mahalanobis gating; the part common to all looks (GNSS, survey) does not shrink. Keep the best single look as a diagnostic. Plan confirming looks in pairs from opposite headings.",
    "F5.lbl": "45°, 100 m, 80% bias, 800 flights", "F5.btn": "Open experiment",

    "F6.title": "The bottom-centre of the box is the near edge of a vehicle, not its centre",
    "F6.where": ["§2.4", "anchor_convention_study.py, deviation 1"],
    "F6.problem": "For an extended object the ray lands on the ground-contact edge nearest the camera. On the bench a 7 m truck is reported 3.5–3.7 m short of its centre at every angle. §2.4 pins “nearest edge” corpus-wide, but the person walking to the vehicle has to know which edge to measure.",
    "F6.fix": "Keep the §2.4 rule and show the operator the look azimuth so the walker measures the same edge; or add a per-class correction to the centre (on real boxes it recovers the centre to 1.0–2.1 m) and pin it in demo-acceptance-harness §4.4.",
    "F6.L": "Object length", "F6.phi": "Angle between object axis and line of sight", "F6.off": "offset to the centre (a bias, not noise)", "F6.tot": "if the centre is measured: r95 at 45° plus the offset", "F6.upto": "up to {x} m",

    "F7.title": "On a slope the terrain lever is set by the incidence angle, not the look angle",
    "F7.where": ["tolerance.terrain_term", "terrain.py"],
    "F7.problem": "The spec and the code use Δh / tan(θ). If the surface falls away from the camera at angle α, the point moves by Δh / (tan θ − tan α): at 25° over a 15° slope that is 5.0 × Δh instead of 2.1 × Δh.",
    "F7.fix": "M-V0-821 returns the surface normal and the ray's incidence angle; M-V0-822 takes the lever from the incidence angle; the refusal threshold is applied to the incidence angle as well.",
    "F7.dep": "Look angle", "F7.slope": "Slope falling away from the camera", "F7.spec": "spec Δh / tan θ", "F7.chain": "chain", "F7.nohit": "no intersection",
    "F7.note": "Incidence angle between ray and surface: {x}°. The factor is metres of horizontal error per metre of terrain height error.",

    

    

    

    "F9.title": "Vertical datum: ellipsoid, geoid or NAVD88",
    "F9.where": ["§2.3", "common.py:14-33 (NED only)", "messages/geodesy.py (111 320 m per degree)"],
    "F9.problem": "The HD40's KLV mixes heights above MSL (tags 15, 25, 42) and above the ellipsoid (75, 78). Public terrain data come in different datums: 3DEP is NAVD88, GLO-30 is EGM2008. A datum mismatch in the US is 20–35 m of height, which becomes Δz / tan(θ) horizontally.",
    "F9.fix": "One vertical datum per mission; every height source is converted into it once (PROJ with a geoid grid). The mission origin is a WGS84 point with a stated realisation; replace the flat conversion with pymap3d or GeographicLib.",
    "F9.dz": "Height error from the datum", "F9.dep": "Look angle", "F9.err": "horizontal error", "F9.under": "aircraft below ground",

    

    

    

    

    "F12.title": "A horizontal survey shift barely matters; the pre-map term mixes two sources",
    "F12.where": ["§1.2 (stale sentence about pre-map matching)", "§4.5 row 3", "tolerance.premap_georef_term"],
    "F12.problem": "PREMAP_GEOREF adds the survey georeferencing error and the aircraft GNSS error one for one. With a GNSS pose, a horizontal shift of the surface does not move the point on a flat field: what matters is its vertical component and the datum. The term overstates one thing and is silent on another, so the “dominant term” points the wrong way.",
    "F12.fix": "Split the contribution into aircraft position (horizontal and vertical) and survey (vertical offset, datum); keep the name PREMAP_GEOREF in the enumeration so the three wires do not change, and pass the breakdown as a share vector.",
    "F12.sh": "Horizontal survey shift", "F12.slope": "Field slope", "F12.spec": "spec", "F12.chain": "chain",

    

    

    

    

    

    

    /* code check tab */
    "ref.title": "Simulator vs reference code",
    "ref.intro": "The vectors below were generated by <code>refgen.py</code>, which calls <code>CameraModel.pixel_rays</code>, <code>cast_sightlines</code> and the term functions of <code>tolerance.py</code>. The page recomputes the same inputs with its own core and shows the difference. While it stays at rounding level, conclusions drawn in the simulator carry over to the code.",
    "ref.cases": "Ray and ground point", "ref.inject": "Injected error of 0.5° and 2.5 m", "ref.terms": "§4.5 terms at the working geometry",
    "ref.uv": "u, v", "ref.pantilt": "pan / tilt", "ref.ypr": "yaw / pitch / roll", "ref.refNE": "reference N, E", "ref.simNE": "simulator N, E", "ref.delta": "Δ, m", "ref.result": "result",
    "ref.match": "match", "ref.differs": "differs",
    "ref.theta": "θ", "ref.source": "source", "ref.refm": "reference, m", "ref.simm": "simulator, m", "ref.specf": "spec formula",
    "ref.term": "term", "ref.simulator": "simulator", "ref.quad": "Total, quadrature",
    "ref.t.pointing": "POINTING (0.5°)", "ref.t.terrain": "TERRAIN (0.3 m)", "ref.t.premap": "PREMAP (3 m + 2 m)", "ref.t.timing": "TIMING (2 ms at 90°/s)",
    "ref.t.camera": "CAMERA_MODEL (0.3 px)", "ref.t.box": "BOX_LOWER_EDGE (5 px)", "ref.dominant": "dominant"
  },
};
