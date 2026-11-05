/* geolocate-detections simulator core.
 * A line-for-line port of akridia_v0_gld conventions:
 *   d_cam   = normalize([(u-cx)/fx, (v-cy)/fy, 1])            camera_model.pixel_rays
 *   d_body  = R_gimbal(pan, tilt) * R_mount * d_cam             sightline.gimbal_rotation
 *   d_world = R_airframe(yaw, pitch, roll) * d_body             ZYX, Body-FRD -> World-NED
 *   origin  = position + R_airframe * lever_arm
 * Spec terms are ported from tolerance.py. The corrected model is a Jacobian
 * covariance of the same chain.
 */
(function (root) {
  "use strict";
  const D = Math.PI / 180;
  const sin = Math.sin, cos = Math.cos, tan = Math.tan;

  const mv = (M, v) => [
    M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
    M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
    M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2],
  ];
  const norm = (v) => { const n = Math.hypot(v[0], v[1], v[2]); return [v[0] / n, v[1] / n, v[2] / n]; };
  const Rx = (a) => [[1, 0, 0], [0, cos(a), -sin(a)], [0, sin(a), cos(a)]];
  const Ry = (a) => [[cos(a), 0, sin(a)], [0, 1, 0], [-sin(a), 0, cos(a)]];
  const Rz = (a) => [[cos(a), -sin(a), 0], [sin(a), cos(a), 0], [0, 0, 1]];
  const mm = (A, B) => A.map((r) => [0, 1, 2].map((j) => r[0] * B[0][j] + r[1] * B[1][j] + r[2] * B[2][j]));

  /** Airframe rotation Body-FRD -> NED. conv 'zyx' is the reference; others are deliberate mistakes. */
  function airframe(yaw, pitch, roll, conv) {
    if (conv === "xyz") return mm(mm(Rx(roll * D), Ry(pitch * D)), Rz(yaw * D));
    if (conv === "yawonly") return Rz(yaw * D);
    return mm(mm(Rz(yaw * D), Ry(pitch * D)), Rx(roll * D));
  }

  /** Camera optical frame -> Body-FRD; columns are image-right, image-down, boresight. */
  function gimbal(pan, tilt) {
    const sp = sin(pan * D), cp = cos(pan * D), st = sin(tilt * D), ct = cos(tilt * D);
    const r = [-sp, cp, 0], d = [st * cp, st * sp, ct], b = [ct * cp, ct * sp, -st];
    return [[r[0], d[0], b[0]], [r[1], d[1], b[1]], [r[2], d[2], b[2]]];
  }

  const focal = (W, hfov) => (W / 2) / tan((hfov * D) / 2);

  function pixelRay(u, v, W, H, hfov) {
    const f = focal(W, hfov);
    return norm([(u - W / 2) / f, (v - H / 2) / f, 1]);
  }

  /** Surface: height above datum z = off + tan(slope) * (p . s), s = unit vector at azimuth slopeAz (uphill). */
  function surfaceHeight(surf, n, e) {
    const k = tan(surf.slope * D), s0 = cos(surf.az * D), s1 = sin(surf.az * D);
    return surf.off + k * ((n - surf.dn) * s0 + (e - surf.de) * s1);
  }

  function intersect(o, d, surf) {
    const k = tan(surf.slope * D), s0 = cos(surf.az * D), s1 = sin(surf.az * D);
    const den = d[2] + k * (d[0] * s0 + d[1] * s1);
    const num = -surf.off - k * ((o[0] - surf.dn) * s0 + (o[1] - surf.de) * s1) - o[2];
    if (den <= 1e-9) return null;
    const t = num / den;
    if (!(t > 0) || t > 1e5) return null;
    const p = [o[0] + t * d[0], o[1] + t * d[1], o[2] + t * d[2]];
    const nrm = norm([k * s0, k * s1, 1]);
    const inc = Math.asin(Math.min(1, Math.abs(d[0] * nrm[0] + d[1] * nrm[1] + d[2] * nrm[2]))) / D;
    return { p, t, inc };
  }

  /** The whole chain for one measurement set m. Returns ray stages and the ground point. */
  function chain(m) {
    const dcam = pixelRay(m.u, m.v, m.W, m.H, m.hfov);
    const G = gimbal(m.pan, m.tilt);
    const dbody = mv(G, dcam);
    const A = airframe(m.yaw, m.pitch, m.roll, m.conv || "zyx");
    const dned = norm(mv(A, dbody));
    const lev = mv(A, m.lever || [0, 0, 0]);
    const o = [m.pos[0] + lev[0], m.pos[1] + lev[1], m.pos[2] + lev[2]];
    const dep = Math.asin(Math.max(-1, Math.min(1, dned[2]))) / D;
    const hit = dep > 0 ? intersect(o, dned, m.surf) : null;
    return { dcam, dbody, dned, o, dep, hit, fx: focal(m.W, m.hfov), az: Math.atan2(dned[1], dned[0]) / D };
  }

  const clone = (m) => ({ ...m, pos: m.pos.slice(), lever: (m.lever || [0, 0, 0]).slice(), surf: { ...m.surf } });

  /* -------- error sources. Each component perturbs the measurement set by x (in its own unit). -------- */
  // sigma(sg) returns this component's one-sigma in the unit apply() expects.
  const COMPONENTS = [
    { src: "heading", grp: "POINTING", bias: true, sigma: (s) => s.heading, apply: (m, x) => { m.yaw += x; } },
    { src: "att", grp: "POINTING", bias: true, sigma: (s) => s.att, apply: (m, x) => { m.pitch += x; } },
    { src: "att", grp: "POINTING", bias: true, sigma: (s) => s.att, apply: (m, x) => { m.roll += x; } },
    { src: "bore", grp: "POINTING", bias: true, sigma: (s) => s.bore, apply: (m, x) => { m.tilt += x; } },
    { src: "bore", grp: "POINTING", bias: true, sigma: (s) => s.bore, apply: (m, x) => { m.pan += x; } },
    { src: "vert", grp: "PREMAP_GEOREF", bias: true, sigma: (s) => s.vert, apply: (m, x) => { m.pos[2] += x; } },
    // gnss and survey are horizontal (radial) sigmas, as sigma_h_m on W-008 and absolute_accuracy_m on W-004
    // carry them; each axis gets sigma / sqrt(2) so the radial RMS equals the stated sigma.
    { src: "gnss", grp: "PREMAP_GEOREF", bias: true, sigma: (s) => s.gnss / Math.SQRT2, apply: (m, x) => { m.pos[0] += x; } },
    { src: "gnss", grp: "PREMAP_GEOREF", bias: true, sigma: (s) => s.gnss / Math.SQRT2, apply: (m, x) => { m.pos[1] += x; } },
    { src: "survey", grp: "PREMAP_GEOREF", bias: true, sigma: (s) => s.survey / Math.SQRT2, apply: (m, x) => { m.surf.dn += x; } },
    { src: "survey", grp: "PREMAP_GEOREF", bias: true, sigma: (s) => s.survey / Math.SQRT2, apply: (m, x) => { m.surf.de += x; } },
    { src: "terrain", grp: "TERRAIN", bias: true, sigma: (s) => s.terrain, apply: (m, x) => { m.surf.off += x; } },
    { src: "dt", grp: "TIMING", bias: false, sigma: (s) => (s.rate * s.dt) / 1000 / Math.SQRT2, apply: (m, x) => { m.tilt += x; } },
    { src: "dt", grp: "TIMING", bias: false, sigma: (s) => (s.rate * s.dt) / 1000 / Math.SQRT2, apply: (m, x) => { m.pan += x; } },
    { src: "dt", grp: "TIMING", bias: false, sigma: (s) => (s.speed * s.dt) / 1000,
      apply: (m, x) => { m.pos[0] += x * cos(m.yaw * D); m.pos[1] += x * sin(m.yaw * D); } },
    { src: "cam", grp: "CAMERA_MODEL", bias: false, sigma: (s) => s.cam, apply: (m, x) => { m.u += x; } },
    { src: "cam", grp: "CAMERA_MODEL", bias: false, sigma: (s) => s.cam, apply: (m, x) => { m.v += x; } },
    { src: "box", grp: "BOX_LOWER_EDGE", bias: false, sigma: (s) => s.box, apply: (m, x) => { m.v += x; } },
  ];
  const GROUPS = ["POINTING", "TERRAIN", "PREMAP_GEOREF", "TIMING", "CAMERA_MODEL", "BOX_LOWER_EDGE"];

  /** Ground displacement for a perturbed measurement: estimate(m') - truth(m). */
  function groundOf(m) { const c = chain(m); return c.hit ? c.hit.p : null; }

  /** Jacobian covariance of the horizontal error, by central differences of the real chain. */
  function jacobianCov(m, sg) {
    const base = groundOf(m);
    if (!base) return null;
    let a = 0, b = 0, c = 0;
    const byGroup = Object.fromEntries(GROUPS.map((g) => [g, 0]));
    const comps = [];
    for (const comp of COMPONENTS) {
      const s = comp.sigma(sg);
      if (!(s > 0)) { comps.push({ ...comp, v: [0, 0] }); continue; }
      const h = s * 0.5;
      const mp = clone(m); comp.apply(mp, h);
      const mn = clone(m); comp.apply(mn, -h);
      const gp = groundOf(mp), gn = groundOf(mn);
      if (!gp || !gn) return null;
      const v = [(gp[0] - gn[0]) / (2 * h) * s, (gp[1] - gn[1]) / (2 * h) * s];
      a += v[0] * v[0]; b += v[0] * v[1]; c += v[1] * v[1];
      byGroup[comp.grp] += v[0] * v[0] + v[1] * v[1];
      comps.push({ ...comp, v });
    }
    return { cov: [[a, b], [b, c]], byGroup, comps, base };
  }

  function eig2(cov) {
    const a = cov[0][0], b = cov[0][1], c = cov[1][1];
    const tr = a + c, det = a * c - b * b, disc = Math.sqrt(Math.max(0, tr * tr / 4 - det));
    const l1 = tr / 2 + disc, l2 = Math.max(0, tr / 2 - disc);
    const ang = Math.abs(b) < 1e-15 ? (a >= c ? 0 : Math.PI / 2) : Math.atan2(l1 - a, b);
    return { l1, l2, ang };
  }

  /** Radius of the circle that holds probability p of a zero-mean 2D Gaussian. */
  function radiusP(cov, p = 0.95) {
    const { l1, l2 } = eig2(cov);
    const s1 = Math.sqrt(l1), s2 = Math.sqrt(Math.max(l2, 1e-12));
    const mass = (r) => {
      const n = 360; let tot = 0;
      for (let i = 0; i < n; i++) {
        const ph = ((i + 0.5) / n) * 2 * Math.PI;
        const q = (cos(ph) / s1) ** 2 + (sin(ph) / s2) ** 2;
        tot += (1 - Math.exp((-q * r * r) / 2)) / q;
      }
      return (tot * (2 * Math.PI / n)) / (2 * Math.PI * s1 * s2);
    };
    let lo = 0, hi = 6 * s1 + 1e-9;
    for (let i = 0; i < 50; i++) { const mid = (lo + hi) / 2; if (mass(mid) < p) lo = mid; else hi = mid; }
    return (lo + hi) / 2;
  }

  /** Along-look / cross-look sigmas for a look azimuth (deg). */
  function alongCross(cov, azDeg) {
    const u = [cos(azDeg * D), sin(azDeg * D)], w = [-u[1], u[0]];
    const q = (x) => x[0] * (cov[0][0] * x[0] + cov[0][1] * x[1]) + x[1] * (cov[1][0] * x[0] + cov[1][1] * x[1]);
    return { along: Math.sqrt(Math.max(0, q(u))), cross: Math.sqrt(Math.max(0, q(w))) };
  }

  /* -------- spec terms, ported from tolerance.py -------- */
  function specTerms(c, m, sg) {
    const slant = c.hit.t;
    const dep = c.dep;
    const hAbove = slant * sin(dep * D);
    const ifov = (m.hfov * D) / m.W;
    const gsdAlong = (hAbove * ifov) / sin(dep * D) ** 2;
    const angle = Math.sqrt(sg.heading ** 2 + 2 * sg.att ** 2 + 2 * sg.bore ** 2);
    const t = {
      POINTING: slant * angle * D,
      TERRAIN: sg.terrain / Math.max(1e-6, Math.abs(tan(dep * D))),
      PREMAP_GEOREF: Math.sqrt(sg.survey ** 2 + sg.gnss ** 2),
      TIMING: slant * tan(((sg.rate * sg.dt) / 1000) * D),
      CAMERA_MODEL: slant * (sg.cam / c.fx),
      BOX_LOWER_EDGE: sg.box * gsdAlong,
    };
    const tol = Math.sqrt(GROUPS.reduce((s, g) => s + t[g] ** 2, 0));
    const dom = GROUPS.reduce((best, g) => (t[g] > t[best] ? g : best), GROUPS[0]);
    return { terms: t, tol, dom, angle, gsdAlong, slant };
  }

  /* -------- random numbers -------- */
  function rng(seed) {
    let a = seed >>> 0;
    const u = () => { a = (a + 0x6d2b79f5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
    let spare = null;
    const g = () => {
      if (spare !== null) { const s = spare; spare = null; return s; }
      let x, y, r; do { x = 2 * u() - 1; y = 2 * u() - 1; r = x * x + y * y; } while (r >= 1 || r === 0);
      const f = Math.sqrt((-2 * Math.log(r)) / r); spare = y * f; return x * f;
    };
    return { u, g };
  }

  /** Monte Carlo through the full chain: every draw perturbs every component. */
  function monteCarlo(m, sg, n, seed) {
    const base = groundOf(m);
    const R = rng(seed);
    const pts = [];
    for (let i = 0; i < n; i++) {
      const mp = clone(m);
      for (const comp of COMPONENTS) { const s = comp.sigma(sg); if (s > 0) comp.apply(mp, R.g() * s); }
      const g = groundOf(mp);
      if (g) pts.push([g[0] - base[0], g[1] - base[1]]);
    }
    return pts;
  }

  const percentile = (arr, p) => { const s = arr.slice().sort((x, y) => x - y); return s[Math.min(s.length - 1, Math.floor(p * s.length))]; };

  /** Look geometry: object at the origin, aircraft placed so the boresight centre hits it from azimuth phi. */
  function lookAt(base, phi) {
    const R = base.h / tan(base.dep * D);
    return {
      u: base.W / 2, v: base.H / 2, W: base.W, H: base.H, hfov: base.hfov,
      pan: 0, tilt: -base.dep, yaw: phi, pitch: 0, roll: 0,
      pos: [-R * cos(phi * D), -R * sin(phi * D), -base.h], lever: [0, 0, 0],
      surf: { off: 0, slope: 0, az: 0, dn: 0, de: 0 }, conv: "zyx",
    };
  }

  const PATTERNS = [
    { id: "one", label: "1 look", az: [0] },
    { id: "same", label: "4 looks, same heading", az: [0, 0, 0, 0] },
    { id: "opp", label: "2 opposite headings", az: [0, 180] },
    { id: "four", label: "4 headings, 90° apart", az: [0, 90, 180, 270] },
    { id: "orbit", label: "orbit, 8 headings", az: [0, 45, 90, 135, 180, 225, 270, 315] },
  ];

  /**
   * Fusion experiment. beta is the share of each biasable source's variance that is a
   * per-sortie bias (identical for every look); the rest is per-look noise.
   */
  function fusionExperiment(base, sg, beta, trials, seed) {
    const R = rng(seed);
    const geo = {}; // per azimuth: measurement set and inverse covariance (weights)
    const allAz = [...new Set(PATTERNS.flatMap((p) => p.az))];
    for (const az of allAz) {
      const m = lookAt(base, az);
      const J = jacobianCov(m, sg);
      const c = J.cov, det = c[0][0] * c[1][1] - c[0][1] * c[1][0];
      geo[az] = { m, W: [[c[1][1] / det, -c[0][1] / det], [-c[1][0] / det, c[0][0] / det]] };
    }
    const out = Object.fromEntries(PATTERNS.map((p) => [p.id, []]));
    const samples = Object.fromEntries(PATTERNS.map((p) => [p.id, []]));
    for (let t = 0; t < trials; t++) {
      const bias = COMPONENTS.map((comp) => (comp.bias ? R.g() * comp.sigma(sg) * Math.sqrt(beta) : 0));
      const cache = {};
      const look = (az, k) => {
        const key = az + ":" + k;
        if (cache[key]) return cache[key];
        const mp = clone(geo[az].m);
        COMPONENTS.forEach((comp, i) => {
          const s = comp.sigma(sg);
          if (!(s > 0)) return;
          const noise = R.g() * s * Math.sqrt(comp.bias ? 1 - beta : 1);
          comp.apply(mp, bias[i] + noise);
        });
        const g = groundOf(mp);
        return (cache[key] = g ? [g[0], g[1]] : [NaN, NaN]);
      };
      for (const p of PATTERNS) {
        let A = [[0, 0], [0, 0]], bvec = [0, 0];
        p.az.forEach((az, k) => {
          const e = look(az, k), Wm = geo[az].W;
          A = [[A[0][0] + Wm[0][0], A[0][1] + Wm[0][1]], [A[1][0] + Wm[1][0], A[1][1] + Wm[1][1]]];
          bvec = [bvec[0] + Wm[0][0] * e[0] + Wm[0][1] * e[1], bvec[1] + Wm[1][0] * e[0] + Wm[1][1] * e[1]];
        });
        const det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
        const f = [(A[1][1] * bvec[0] - A[0][1] * bvec[1]) / det, (-A[1][0] * bvec[0] + A[0][0] * bvec[1]) / det];
        out[p.id].push(Math.hypot(f[0], f[1]));
        if (samples[p.id].length < 600) samples[p.id].push(f);
      }
    }
    const r95 = Object.fromEntries(PATTERNS.map((p) => [p.id, percentile(out[p.id], 0.95)]));
    return { r95, samples };
  }

  root.GLD = {
    D, chain, clone, intersect, surfaceHeight, pixelRay, gimbal, airframe, focal,
    COMPONENTS, GROUPS, jacobianCov, eig2, radiusP, alongCross, specTerms,
    rng, monteCarlo, percentile, lookAt, PATTERNS, fusionExperiment, groundOf,
  };
})(typeof window !== "undefined" ? window : globalThis);
