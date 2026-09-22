// Narrow phase. Produces manifolds of at most two contact points, each carrying
// a stable feature id so the solver can warm-start it on the next step.

import * as m from './math.js';
import { CIRCLE, POLYGON } from './body.js';

/**
 * Feature ids pack the edges that produced a contact plus its ordinal along the
 * reference face. The ordinal matters: clipping can hand both surviving points
 * the same originating edge, and two contacts sharing an id make the solver
 * warm-start them from the same stored impulse -- which loads one side of a
 * resting box harder than the other and tips a tall stack over.
 */
function featureId(refEdge, incEdge, flipped, ordinal = 0) {
  return (
    (refEdge & 0xff) |
    ((incEdge & 0xff) << 8) |
    ((ordinal & 0x3) << 16) |
    (flipped ? 0x40000 : 0)
  );
}

export function aabbOverlap(a, b, margin = 0) {
  return (
    a.maxX + margin >= b.minX &&
    a.minX - margin <= b.maxX &&
    a.maxY + margin >= b.minY &&
    a.minY - margin <= b.maxY
  );
}

/**
 * Collide two bodies. Returns null when they are separated, otherwise
 * `{ normal, contacts }` where `normal` points from `a` towards `b` and each
 * contact has `{ point, penetration, id }` with penetration > 0.
 */
export function collide(a, b) {
  if (a.shape.type === CIRCLE && b.shape.type === CIRCLE) return circleCircle(a, b);
  if (a.shape.type === POLYGON && b.shape.type === CIRCLE) return polygonCircle(a, b, false);
  if (a.shape.type === CIRCLE && b.shape.type === POLYGON) return polygonCircle(b, a, true);
  return polygonPolygon(a, b);
}

function circleCircle(a, b) {
  const d = m.sub(b.position, a.position);
  const r = a.shape.radius + b.shape.radius;
  const distSq = m.lenSq(d);
  if (distSq >= r * r) return null;

  const distance = Math.sqrt(distSq);
  // Perfectly coincident centres have no separating direction; pick one so the
  // pair still resolves instead of producing a NaN normal.
  const normal = distance > m.EPS ? m.scale(d, 1 / distance) : m.v(0, -1);
  const penetration = r - distance;
  const point = m.add(a.position, m.scale(normal, a.shape.radius - penetration / 2));
  return { normal, contacts: [{ point, penetration, id: 0 }] };
}

/**
 * Polygon against circle. `flipped` records that the caller's `a` was the
 * circle, so the returned normal can be reversed back to a -> b.
 */
function polygonCircle(poly, circ, flipped) {
  const center = poly.toLocal(circ.position);
  const radius = circ.shape.radius;
  const verts = poly.shape.vertices;
  const normals = poly.shape.normals;

  // Deepest face, in the polygon's local frame.
  let bestIndex = 0;
  let bestSep = -Infinity;
  for (let i = 0; i < verts.length; i++) {
    const s = m.dot(normals[i], m.sub(center, verts[i]));
    if (s > radius) return null; // separating axis
    if (s > bestSep) {
      bestSep = s;
      bestIndex = i;
    }
  }

  const i1 = bestIndex;
  const i2 = (bestIndex + 1) % verts.length;
  const v1 = verts[i1];
  const v2 = verts[i2];

  let localNormal;
  let localPoint;
  if (bestSep < m.EPS) {
    // Centre is inside the polygon: push out along the shallowest face.
    localNormal = normals[i1];
    localPoint = m.add(center, m.scale(localNormal, -bestSep));
  } else {
    const u1 = m.dot(m.sub(center, v1), m.sub(v2, v1));
    const u2 = m.dot(m.sub(center, v2), m.sub(v1, v2));
    if (u1 <= 0) {
      if (m.lenSq(m.sub(center, v1)) > radius * radius) return null;
      localNormal = m.normalize(m.sub(center, v1));
      localPoint = v1;
    } else if (u2 <= 0) {
      if (m.lenSq(m.sub(center, v2)) > radius * radius) return null;
      localNormal = m.normalize(m.sub(center, v2));
      localPoint = v2;
    } else {
      localNormal = normals[i1];
      if (m.dot(m.sub(center, v1), localNormal) > radius) return null;
      localPoint = m.add(center, m.scale(localNormal, -bestSep));
    }
  }

  const penetration = radius - m.dot(m.sub(center, localPoint), localNormal);
  if (penetration <= 0) return null;

  // Normal currently points polygon -> circle.
  let normal = m.rotate(localNormal, poly.angle);
  const point = poly.toWorld(localPoint);
  if (flipped) normal = m.neg(normal);
  return { normal, contacts: [{ point, penetration, id: featureId(i1, 0, flipped) }] };
}

/** Deepest penetration of `b` into any face of `a`. Negative means separated. */
function maxSeparation(a, b) {
  const aVerts = a.shape.vertices;
  const aNormals = a.shape.normals;
  let bestIndex = 0;
  let bestSep = -Infinity;

  for (let i = 0; i < aVerts.length; i++) {
    const nWorld = m.rotate(aNormals[i], a.angle);
    const nInB = m.unrotate(nWorld, b.angle);
    // Support point of b in the direction opposite the face normal.
    let support = Infinity;
    let supportPoint = null;
    for (const p of b.shape.vertices) {
      const proj = m.dot(p, nInB);
      if (proj < support) {
        support = proj;
        supportPoint = p;
      }
    }
    const faceWorld = a.toWorld(aVerts[i]);
    const sep = m.dot(m.sub(b.toWorld(supportPoint), faceWorld), nWorld);
    if (sep > bestSep) {
      bestSep = sep;
      bestIndex = i;
    }
  }
  return { separation: bestSep, index: bestIndex };
}

/** The face of `inc` most anti-parallel to the reference normal. */
function incidentEdge(refNormalWorld, inc) {
  const normals = inc.shape.normals;
  const nInInc = m.unrotate(refNormalWorld, inc.angle);
  let bestIndex = 0;
  let bestDot = Infinity;
  for (let i = 0; i < normals.length; i++) {
    const d = m.dot(normals[i], nInInc);
    if (d < bestDot) {
      bestDot = d;
      bestIndex = i;
    }
  }
  const i2 = (bestIndex + 1) % inc.shape.vertices.length;
  return {
    index: bestIndex,
    points: [inc.toWorld(inc.shape.vertices[bestIndex]), inc.toWorld(inc.shape.vertices[i2])],
  };
}

/** Clip a segment against a half-plane, keeping the side where dot(n, p) <= offset. */
function clipSegment(points, normal, offset, ids) {
  const out = [];
  const outIds = [];
  const d0 = m.dot(normal, points[0]) - offset;
  const d1 = m.dot(normal, points[1]) - offset;

  if (d0 <= 0) {
    out.push(points[0]);
    outIds.push(ids[0]);
  }
  if (d1 <= 0) {
    out.push(points[1]);
    outIds.push(ids[1]);
  }
  if (d0 * d1 < 0) {
    const t = d0 / (d0 - d1);
    out.push(m.add(points[0], m.scale(m.sub(points[1], points[0]), t)));
    outIds.push(ids[0]);
  }
  return { points: out.slice(0, 2), ids: outIds.slice(0, 2) };
}

function polygonPolygon(a, b) {
  const sepA = maxSeparation(a, b);
  if (sepA.separation > 0) return null;
  const sepB = maxSeparation(b, a);
  if (sepB.separation > 0) return null;

  // Prefer A's axis unless B's is meaningfully deeper, which keeps the choice
  // stable frame to frame and stops warm-start impulses being thrown away.
  const flipped = sepB.separation > sepA.separation + 0.1 * Math.abs(sepA.separation) + 1e-4;
  const ref = flipped ? b : a;
  const inc = flipped ? a : b;
  const refIndex = flipped ? sepB.index : sepA.index;

  const refVerts = ref.shape.vertices;
  const i1 = refIndex;
  const i2 = (refIndex + 1) % refVerts.length;
  const p1 = ref.toWorld(refVerts[i1]);
  const p2 = ref.toWorld(refVerts[i2]);
  const refNormal = m.rotate(ref.shape.normals[i1], ref.angle);
  const tangent = m.normalize(m.sub(p2, p1));

  const incident = incidentEdge(refNormal, inc);
  let clip = clipSegment(incident.points, m.neg(tangent), -m.dot(tangent, p1), [
    featureId(i1, incident.index, flipped),
    featureId(i1, (incident.index + 1) % inc.shape.vertices.length, flipped),
  ]);
  if (clip.points.length < 2) return null;
  clip = clipSegment(clip.points, tangent, m.dot(tangent, p2), clip.ids);
  if (clip.points.length < 2) return null;

  const frontOffset = m.dot(refNormal, p1);
  const contacts = [];
  for (let i = 0; i < clip.points.length; i++) {
    const separation = m.dot(refNormal, clip.points[i]) - frontOffset;
    if (separation <= 0) {
      contacts.push({
        // Project onto the reference face so both points share a plane; this is
        // what keeps a resting box from rocking on one corner.
        point: m.sub(clip.points[i], m.scale(refNormal, separation * 0.5)),
        penetration: -separation,
        id: clip.ids[i],
        along: m.dot(tangent, clip.points[i]),
      });
    }
  }
  if (contacts.length === 0) return null;

  // Order along the reference face, then stamp the ordinal in. Position along
  // the face is what stays stable as the incident body slides.
  contacts.sort((x, y) => x.along - y.along);
  for (let i = 0; i < contacts.length; i++) {
    contacts[i].id = featureId(i1, incident.index, flipped, i);
    delete contacts[i].along;
  }

  // refNormal points ref -> inc; the caller wants a -> b.
  return { normal: flipped ? m.neg(refNormal) : refNormal, contacts };
}

/** Ray cast against one body. Returns `{ t, point, normal }` or null. */
export function raycastBody(body, origin, direction, maxDistance = Infinity) {
  const dir = m.normalize(direction);
  if (body.shape.type === CIRCLE) {
    const oc = m.sub(origin, body.position);
    const b = m.dot(oc, dir);
    const c = m.lenSq(oc) - body.shape.radius ** 2;
    const disc = b * b - c;
    if (disc < 0) return null;
    const t = -b - Math.sqrt(disc);
    if (t < 0 || t > maxDistance) return null;
    const point = m.add(origin, m.scale(dir, t));
    return { t, point, normal: m.normalize(m.sub(point, body.position)) };
  }

  // Slab clipping in the polygon's local frame.
  const p = body.toLocal(origin);
  const d = m.unrotate(dir, body.angle);
  let lower = 0;
  let upper = maxDistance;
  let index = -1;
  const verts = body.shape.vertices;
  const normals = body.shape.normals;

  for (let i = 0; i < verts.length; i++) {
    const numerator = m.dot(normals[i], m.sub(verts[i], p));
    const denominator = m.dot(normals[i], d);
    if (Math.abs(denominator) < m.EPS) {
      if (numerator < 0) return null;
    } else {
      const t = numerator / denominator;
      if (denominator < 0 && t > lower) {
        lower = t;
        index = i;
      } else if (denominator > 0 && t < upper) {
        upper = t;
      }
    }
    if (upper < lower) return null;
  }
  if (index < 0) return null;
  return {
    t: lower,
    point: m.add(origin, m.scale(dir, lower)),
    normal: m.rotate(normals[index], body.angle),
  };
}
