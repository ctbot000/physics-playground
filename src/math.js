// 2D vector math. Vectors are plain {x, y} objects; every function is pure.

export const EPS = 1e-9;

export function v(x = 0, y = 0) {
  return { x, y };
}

export function clone(a) {
  return { x: a.x, y: a.y };
}

export function add(a, b) {
  return { x: a.x + b.x, y: a.y + b.y };
}

export function sub(a, b) {
  return { x: a.x - b.x, y: a.y - b.y };
}

export function scale(a, s) {
  return { x: a.x * s, y: a.y * s };
}

export function neg(a) {
  return { x: -a.x, y: -a.y };
}

export function dot(a, b) {
  return a.x * b.x + a.y * b.y;
}

/** Scalar z-component of the 3D cross product of two planar vectors. */
export function cross(a, b) {
  return a.x * b.y - a.y * b.x;
}

/** v x s -- a vector crossed with a scalar angular quantity. */
export function crossVS(a, s) {
  return { x: s * a.y, y: -s * a.x };
}

/** s x v -- a scalar angular quantity crossed with a vector. */
export function crossSV(s, a) {
  return { x: -s * a.y, y: s * a.x };
}

export function len(a) {
  return Math.hypot(a.x, a.y);
}

export function lenSq(a) {
  return a.x * a.x + a.y * a.y;
}

export function dist(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

export function normalize(a) {
  const l = Math.hypot(a.x, a.y);
  return l > EPS ? { x: a.x / l, y: a.y / l } : { x: 0, y: 0 };
}

/** Perpendicular rotated +90 degrees. */
export function perp(a) {
  return { x: -a.y, y: a.x };
}

/**
 * Perpendicular rotated -90 degrees. For an edge p->q of a counter-clockwise
 * polygon this is the *outward* normal; `perp` gives the inward one.
 */
export function rperp(a) {
  return { x: a.y, y: -a.x };
}

export function rotate(a, angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return { x: a.x * c - a.y * s, y: a.x * s + a.y * c };
}

/** Rotate by -angle: takes a world-space offset into a body's local frame. */
export function unrotate(a, angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return { x: a.x * c + a.y * s, y: -a.x * s + a.y * c };
}

export function lerp(a, b, t) {
  return a + (b - a) * t;
}

export function clamp(x, lo, hi) {
  return x < lo ? lo : x > hi ? hi : x;
}

export function isFiniteVec(a) {
  return Number.isFinite(a.x) && Number.isFinite(a.y);
}

/**
 * Invert a symmetric 2x2 block [[a, b], [b, d]].
 * Returns null when the block is singular, so callers can skip rather than
 * propagate an Infinity into the solver state.
 */
export function invert2x2(a, b, d) {
  const det = a * d - b * b;
  if (Math.abs(det) < EPS) return null;
  const inv = 1 / det;
  return { a: d * inv, b: -b * inv, d: a * inv };
}

/** Deterministic PRNG so a scene can be replayed from its seed. */
export function mulberry32(seed) {
  let t = seed >>> 0;
  return function random() {
    t = (t + 0x6d2b79f5) >>> 0;
    let x = Math.imul(t ^ (t >>> 15), 1 | t);
    x = (x + Math.imul(x ^ (x >>> 7), 61 | x)) ^ x;
    return ((x ^ (x >>> 14)) >>> 0) / 4294967296;
  };
}
