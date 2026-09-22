// Rigid bodies: a shape, a transform, and the mass properties derived from them.

import * as m from './math.js';

let nextId = 1;

export const CIRCLE = 'circle';
export const POLYGON = 'polygon';

/** Shape factory: a disc of the given radius, centred on the body origin. */
export function circleShape(radius) {
  return { type: CIRCLE, radius: Math.max(radius, 0.01) };
}

/**
 * Shape factory: a convex polygon. Vertices may be given in either winding and
 * anywhere in the plane -- they are hulled, wound counter-clockwise and
 * recentred on the area centroid, so the body origin is always the centre of
 * mass. `offset` reports how far the vertices moved, so a caller that placed
 * them in world coordinates can correct the body position.
 */
export function polygonShape(points) {
  const hull = convexHull(points);
  if (hull.length < 3) throw new Error('polygon needs at least 3 distinct vertices');
  const c = polygonCentroid(hull);
  const vertices = hull.map((p) => ({ x: p.x - c.x, y: p.y - c.y }));
  // Outward normals. `perp` would give the inward one, which every containment
  // and separating-axis test below silently inverts.
  const normals = vertices.map((p, i) => {
    const q = vertices[(i + 1) % vertices.length];
    return m.normalize(m.rperp(m.sub(q, p)));
  });
  let radius = 0;
  for (const p of vertices) radius = Math.max(radius, m.len(p));
  return { type: POLYGON, vertices, normals, radius, offset: c };
}

export function boxShape(halfWidth, halfHeight) {
  return polygonShape([
    { x: -halfWidth, y: -halfHeight },
    { x: halfWidth, y: -halfHeight },
    { x: halfWidth, y: halfHeight },
    { x: -halfWidth, y: halfHeight },
  ]);
}

export function regularPolygonShape(sides, radius, phase = 0) {
  const pts = [];
  for (let i = 0; i < sides; i++) {
    const a = phase + (i / sides) * Math.PI * 2;
    pts.push({ x: Math.cos(a) * radius, y: Math.sin(a) * radius });
  }
  return polygonShape(pts);
}

/** Andrew's monotone chain, counter-clockwise, duplicate points dropped. */
export function convexHull(points) {
  const pts = points
    .map((p) => ({ x: p.x, y: p.y }))
    .sort((a, b) => (a.x === b.x ? a.y - b.y : a.x - b.x))
    .filter((p, i, arr) => i === 0 || Math.abs(p.x - arr[i - 1].x) > m.EPS || Math.abs(p.y - arr[i - 1].y) > m.EPS);
  if (pts.length < 3) return pts;

  const half = (source) => {
    const out = [];
    for (const p of source) {
      while (out.length >= 2 && m.cross(m.sub(out[out.length - 1], out[out.length - 2]), m.sub(p, out[out.length - 2])) <= 0) {
        out.pop();
      }
      out.push(p);
    }
    out.pop();
    return out;
  };

  return [...half(pts), ...half([...pts].reverse())];
}

export function polygonArea(vertices) {
  let a = 0;
  for (let i = 0; i < vertices.length; i++) {
    const p = vertices[i];
    const q = vertices[(i + 1) % vertices.length];
    a += m.cross(p, q);
  }
  return a / 2;
}

export function polygonCentroid(vertices) {
  let a = 0;
  let cx = 0;
  let cy = 0;
  for (let i = 0; i < vertices.length; i++) {
    const p = vertices[i];
    const q = vertices[(i + 1) % vertices.length];
    const w = m.cross(p, q);
    a += w;
    cx += (p.x + q.x) * w;
    cy += (p.y + q.y) * w;
  }
  if (Math.abs(a) < m.EPS) {
    // Degenerate ring: fall back to the vertex average rather than dividing by ~0.
    const n = vertices.length;
    return {
      x: vertices.reduce((s, p) => s + p.x, 0) / n,
      y: vertices.reduce((s, p) => s + p.y, 0) / n,
    };
  }
  return { x: cx / (3 * a), y: cy / (3 * a) };
}

/**
 * Second moment of area about the centroid, for vertices already centred on it.
 * Multiplied by density this is the rotational inertia.
 */
export function polygonMomentOfArea(vertices) {
  let num = 0;
  let den = 0;
  for (let i = 0; i < vertices.length; i++) {
    const p = vertices[i];
    const q = vertices[(i + 1) % vertices.length];
    const w = Math.abs(m.cross(p, q));
    num += w * (m.dot(p, p) + m.dot(p, q) + m.dot(q, q));
    den += w;
  }
  return den < m.EPS ? 0 : num / 12;
}

export class Body {
  constructor(options = {}) {
    const {
      shape,
      position = m.v(0, 0),
      angle = 0,
      velocity = m.v(0, 0),
      angularVelocity = 0,
      density = 1,
      restitution = 0.2,
      friction = 0.4,
      isStatic = false,
      color = null,
      label = '',
      tag = null,
    } = options;

    if (!shape) throw new Error('Body requires a shape');

    this.id = nextId++;
    this.shape = shape;
    this.position = m.clone(position);
    this.angle = angle;
    this.velocity = m.clone(velocity);
    this.angularVelocity = angularVelocity;
    this.force = m.v(0, 0);
    this.torque = 0;

    this.density = density;
    this.restitution = restitution;
    this.friction = friction;
    this.isStatic = isStatic;
    this.color = color;
    this.label = label;
    this.tag = tag;

    // Per-body overrides; null means "use the world value".
    this.gravityScale = 1;
    this.linearDamping = null;
    this.angularDamping = null;

    this.mass = 0;
    this.invMass = 0;
    this.inertia = 0;
    this.invInertia = 0;
    this.updateMass();

    this.aabb = { minX: 0, minY: 0, maxX: 0, maxY: 0 };
    this.updateAABB();
  }

  get area() {
    return this.shape.type === CIRCLE
      ? Math.PI * this.shape.radius * this.shape.radius
      : Math.abs(polygonArea(this.shape.vertices));
  }

  /** Largest distance from the body origin to any point of the shape. */
  get boundingRadius() {
    return this.shape.type === CIRCLE ? this.shape.radius : this.shape.radius;
  }

  updateMass() {
    if (this.isStatic) {
      this.mass = 0;
      this.invMass = 0;
      this.inertia = 0;
      this.invInertia = 0;
      return;
    }
    const area = this.area;
    this.mass = this.density * area;
    this.invMass = this.mass > m.EPS ? 1 / this.mass : 0;
    if (this.shape.type === CIRCLE) {
      this.inertia = 0.5 * this.mass * this.shape.radius * this.shape.radius;
    } else {
      this.inertia = this.density * polygonMomentOfArea(this.shape.vertices);
    }
    this.invInertia = this.inertia > m.EPS ? 1 / this.inertia : 0;
  }

  setStatic(isStatic) {
    this.isStatic = isStatic;
    if (isStatic) {
      this.velocity = m.v(0, 0);
      this.angularVelocity = 0;
    }
    this.updateMass();
  }

  /** Local-space point to world space. */
  toWorld(point) {
    return m.add(this.position, m.rotate(point, this.angle));
  }

  /** World-space point to this body's local frame. */
  toLocal(point) {
    return m.unrotate(m.sub(point, this.position), this.angle);
  }

  /** Velocity of the material point currently at the given world position. */
  velocityAt(worldPoint) {
    const r = m.sub(worldPoint, this.position);
    return m.add(this.velocity, m.crossSV(this.angularVelocity, r));
  }

  worldVertices() {
    if (this.shape.type !== POLYGON) return [];
    return this.shape.vertices.map((p) => this.toWorld(p));
  }

  worldNormals() {
    if (this.shape.type !== POLYGON) return [];
    return this.shape.normals.map((n) => m.rotate(n, this.angle));
  }

  applyForce(force, worldPoint = null) {
    this.force = m.add(this.force, force);
    if (worldPoint) this.torque += m.cross(m.sub(worldPoint, this.position), force);
  }

  applyImpulse(impulse, worldPoint = null) {
    if (this.isStatic) return;
    this.velocity = m.add(this.velocity, m.scale(impulse, this.invMass));
    if (worldPoint) {
      this.angularVelocity += this.invInertia * m.cross(m.sub(worldPoint, this.position), impulse);
    }
  }

  clearForces() {
    this.force = m.v(0, 0);
    this.torque = 0;
  }

  kineticEnergy() {
    if (this.isStatic) return 0;
    return 0.5 * this.mass * m.lenSq(this.velocity) + 0.5 * this.inertia * this.angularVelocity ** 2;
  }

  updateAABB() {
    if (this.shape.type === CIRCLE) {
      const r = this.shape.radius;
      this.aabb.minX = this.position.x - r;
      this.aabb.minY = this.position.y - r;
      this.aabb.maxX = this.position.x + r;
      this.aabb.maxY = this.position.y + r;
      return this.aabb;
    }
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    const c = Math.cos(this.angle);
    const s = Math.sin(this.angle);
    for (const p of this.shape.vertices) {
      const x = this.position.x + p.x * c - p.y * s;
      const y = this.position.y + p.x * s + p.y * c;
      if (x < minX) minX = x;
      if (y < minY) minY = y;
      if (x > maxX) maxX = x;
      if (y > maxY) maxY = y;
    }
    this.aabb.minX = minX;
    this.aabb.minY = minY;
    this.aabb.maxX = maxX;
    this.aabb.maxY = maxY;
    return this.aabb;
  }

  containsPoint(worldPoint) {
    if (this.shape.type === CIRCLE) {
      return m.lenSq(m.sub(worldPoint, this.position)) <= this.shape.radius ** 2;
    }
    const local = this.toLocal(worldPoint);
    const verts = this.shape.vertices;
    for (let i = 0; i < verts.length; i++) {
      if (m.dot(this.shape.normals[i], m.sub(local, verts[i])) > 0) return false;
    }
    return true;
  }

  /**
   * Non-finite state is contagious: one NaN spreads through every contact the
   * body takes part in, and a clamped step count derived from it silently runs
   * zero iterations. Park the body at rest instead of letting it propagate.
   */
  sanitize() {
    if (!m.isFiniteVec(this.position) || !Number.isFinite(this.angle)) {
      this.position = m.v(0, 0);
      this.angle = 0;
      this.velocity = m.v(0, 0);
      this.angularVelocity = 0;
      return false;
    }
    if (!m.isFiniteVec(this.velocity)) this.velocity = m.v(0, 0);
    if (!Number.isFinite(this.angularVelocity)) this.angularVelocity = 0;
    return true;
  }
}

/** Convenience constructors. */
export function circle(x, y, radius, options = {}) {
  return new Body({ ...options, shape: circleShape(radius), position: m.v(x, y) });
}

export function box(x, y, halfWidth, halfHeight, options = {}) {
  return new Body({ ...options, shape: boxShape(halfWidth, halfHeight), position: m.v(x, y) });
}

export function polygon(x, y, points, options = {}) {
  return new Body({ ...options, shape: polygonShape(points), position: m.v(x, y) });
}

export function ngon(x, y, sides, radius, options = {}) {
  return new Body({
    ...options,
    shape: regularPolygonShape(sides, radius, options.phase ?? 0),
    position: m.v(x, y),
  });
}
