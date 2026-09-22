// The simulation. Fixed-timestep, impulse-based sequential solver with warm
// starting; contacts persist between steps so stacks converge.

import * as m from './math.js';
import { collide, aabbOverlap, raycastBody } from './collide.js';
import { MouseJoint } from './joints.js';

/** One persistent contact pair. Accumulated impulses survive across steps. */
class Arbiter {
  constructor(a, b) {
    this.a = a;
    this.b = b;
    this.key = pairKey(a, b);
    this.contacts = [];
    this.normal = m.v(0, 1);
    // Geometric means/maxima are the conventional mixing rules: a slippery body
    // stays slippery against anything, and the bouncier body sets the bounce.
    this.friction = Math.sqrt(a.friction * b.friction);
    this.restitution = Math.max(a.restitution, b.restitution);
    this.touching = false;
    this.maxImpulse = 0;
  }

  /** Merge a fresh manifold, carrying accumulated impulses over by feature id. */
  update(manifold) {
    const previous = this.contacts;
    this.normal = manifold.normal;
    this.contacts = manifold.contacts.map((c) => {
      const old = previous.find((p) => p.id === c.id);
      return {
        point: c.point,
        penetration: c.penetration,
        id: c.id,
        // Anchors in each body's own frame, so the position pass can recompute
        // the separation from the transforms it is busy changing.
        localA: this.a.toLocal(c.point),
        localB: this.b.toLocal(c.point),
        separation0: -c.penetration,
        normalImpulse: old ? old.normalImpulse : 0,
        tangentImpulse: old ? old.tangentImpulse : 0,
        r1: m.v(0, 0),
        r2: m.v(0, 0),
        massNormal: 0,
        massTangent: 0,
        restitutionBias: 0,
      };
    });
    this.touching = this.contacts.length > 0;
    this.localNormal = m.unrotate(manifold.normal, this.a.angle);
  }

  preStep(invDt, config) {
    const { a, b } = this;
    const tangent = m.perp(this.normal);

    for (const c of this.contacts) {
      c.r1 = m.sub(c.point, a.position);
      c.r2 = m.sub(c.point, b.position);

      const rn1 = m.cross(c.r1, this.normal);
      const rn2 = m.cross(c.r2, this.normal);
      const kNormal = a.invMass + b.invMass + a.invInertia * rn1 * rn1 + b.invInertia * rn2 * rn2;
      c.massNormal = kNormal > m.EPS ? 1 / kNormal : 0;

      const rt1 = m.cross(c.r1, tangent);
      const rt2 = m.cross(c.r2, tangent);
      const kTangent = a.invMass + b.invMass + a.invInertia * rt1 * rt1 + b.invInertia * rt2 * rt2;
      c.massTangent = kTangent > m.EPS ? 1 / kTangent : 0;

      // Overlap is removed by the position pass, not by a velocity bias. A
      // bias pushes bodies apart by adding real velocity, and over repeated
      // bounces that energy accumulates: a perfectly elastic ball climbs
      // higher than it was dropped from, and a stack slowly shakes itself
      // apart. Correcting position directly adds none.

      // Restitution is evaluated once, against the approach speed before the
      // solver runs; doing it per iteration would add energy.
      const dv = relativeVelocity(a, b, c.r1, c.r2);
      const vn = m.dot(dv, this.normal);
      const approach = -vn;
      if (approach > config.restitutionThreshold) {
        // A discrete step overshoots the surface, and the body keeps
        // accelerating over that extra depth. Reflecting the speed measured at
        // the bottom of the overlap therefore returns more than went in, and
        // the position pass then hands the depth back as height: an elastic
        // ball climbs a little higher on every bounce. Discount the speed
        // gravity added while the body was below the surface.
        const gn = Math.abs(m.dot(config.gravity, this.normal));
        // Gravity for this step has already been integrated into the velocity
        // above, and the body has been below the surface for penetration/speed
        // before that; discount both.
        const submergedTime = c.penetration / approach + config.dt;
        const atSurface = Math.max(0, approach - gn * submergedTime);
        c.restitutionBias = this.restitution * atSurface;
      } else {
        c.restitutionBias = 0;
      }

      if (config.warmStarting) {
        const p = m.add(m.scale(this.normal, c.normalImpulse), m.scale(tangent, c.tangentImpulse));
        applyPairImpulse(a, b, p, c.r1, c.r2);
      } else {
        c.normalImpulse = 0;
        c.tangentImpulse = 0;
      }
    }
  }

  /**
   * Push overlapping bodies apart by moving them, not by giving them velocity.
   * Returns the deepest remaining overlap so the caller can stop early.
   */
  solvePosition(config) {
    const { a, b } = this;
    let worst = 0;
    const normal = m.rotate(this.localNormal, a.angle);

    for (const c of this.contacts) {
      const pA = a.toWorld(c.localA);
      const pB = b.toWorld(c.localB);
      const separation = m.dot(m.sub(pB, pA), normal) + c.separation0;
      worst = Math.min(worst, separation);

      const rA = m.sub(pA, a.position);
      const rB = m.sub(pB, b.position);
      const rnA = m.cross(rA, normal);
      const rnB = m.cross(rB, normal);
      const k = a.invMass + b.invMass + a.invInertia * rnA * rnA + b.invInertia * rnB * rnB;
      if (k <= m.EPS) continue;

      // Correct a fraction of the error per iteration, and never more than
      // maxCorrection at once, so a deep overlap unwinds smoothly.
      const correction = m.clamp(
        config.biasFactor * (separation + config.allowedPenetration),
        -config.maxCorrection,
        0,
      );
      if (correction === 0) continue;
      const impulse = -correction / k;
      const p = m.scale(normal, impulse);

      if (!a.isStatic) {
        a.position = m.sub(a.position, m.scale(p, a.invMass));
        a.angle -= a.invInertia * m.cross(rA, p);
      }
      if (!b.isStatic) {
        b.position = m.add(b.position, m.scale(p, b.invMass));
        b.angle += b.invInertia * m.cross(rB, p);
      }
    }
    return worst;
  }

  applyImpulse() {
    const { a, b } = this;
    const tangent = m.perp(this.normal);
    let peak = 0;

    for (const c of this.contacts) {
      // --- Normal: non-penetration, impulse clamped to be repulsive only. ---
      let dv = relativeVelocity(a, b, c.r1, c.r2);
      const vn = m.dot(dv, this.normal);
      let dPn = c.massNormal * (-vn + c.restitutionBias);
      const oldPn = c.normalImpulse;
      c.normalImpulse = Math.max(oldPn + dPn, 0);
      dPn = c.normalImpulse - oldPn;
      applyPairImpulse(a, b, m.scale(this.normal, dPn), c.r1, c.r2);

      // --- Friction: Coulomb, bounded by the normal impulse of this step. ---
      // Scaling tangential velocity by a flat coefficient instead would bleed a
      // rolling body to a halt, because a resting contact is re-solved every
      // step and the loss compounds at the substep rate.
      dv = relativeVelocity(a, b, c.r1, c.r2);
      const vt = m.dot(dv, tangent);
      let dPt = c.massTangent * -vt;
      const maxPt = this.friction * c.normalImpulse;
      const oldPt = c.tangentImpulse;
      c.tangentImpulse = m.clamp(oldPt + dPt, -maxPt, maxPt);
      dPt = c.tangentImpulse - oldPt;
      applyPairImpulse(a, b, m.scale(tangent, dPt), c.r1, c.r2);

      if (c.normalImpulse > peak) peak = c.normalImpulse;
    }
    this.maxImpulse = peak;
  }
}

function relativeVelocity(a, b, r1, r2) {
  return m.sub(
    m.add(b.velocity, m.crossSV(b.angularVelocity, r2)),
    m.add(a.velocity, m.crossSV(a.angularVelocity, r1)),
  );
}

function applyPairImpulse(a, b, impulse, r1, r2) {
  if (!a.isStatic) {
    a.velocity = m.sub(a.velocity, m.scale(impulse, a.invMass));
    a.angularVelocity -= a.invInertia * m.cross(r1, impulse);
  }
  if (!b.isStatic) {
    b.velocity = m.add(b.velocity, m.scale(impulse, b.invMass));
    b.angularVelocity += b.invInertia * m.cross(r2, impulse);
  }
}

function pairKey(a, b) {
  return a.id < b.id ? `${a.id}:${b.id}` : `${b.id}:${a.id}`;
}

export class World {
  constructor(options = {}) {
    this.bodies = [];
    this.joints = [];
    this.arbiters = new Map();

    this.gravity = options.gravity ? m.clone(options.gravity) : m.v(0, 900);
    this.linearDamping = options.linearDamping ?? 0.02;
    this.angularDamping = options.angularDamping ?? 0.02;
    this.velocityIterations = options.velocityIterations ?? 10;
    this.positionIterations = options.positionIterations ?? 4;
    this.warmStarting = options.warmStarting ?? true;
    this.allowedPenetration = options.allowedPenetration ?? 0.4;
    this.biasFactor = options.biasFactor ?? 0.25;
    this.maxCorrection = options.maxCorrection ?? 6;
    this.restitutionThreshold = options.restitutionThreshold ?? 40;
    this.fixedDt = options.fixedDt ?? 1 / 120;
    this.maxSubSteps = options.maxSubSteps ?? 6;

    // World rectangle. Bodies are clamped back inside it after every step, as a
    // backstop behind the static walls the scenes build.
    this.bounds = options.bounds ?? { x: 0, y: 0, width: 1200, height: 760 };
    this.clampToBounds = options.clampToBounds ?? true;

    this.accumulator = 0;
    this.time = 0;
    this.stepCount = 0;
    this.onPreStep = null;
    this.impacts = [];
    this.listeners = { collision: [] };
    this.stats = { pairs: 0, contacts: 0, broadPhaseChecks: 0 };
  }

  add(body) {
    this.bodies.push(body);
    return body;
  }

  addAll(bodies) {
    for (const b of bodies) this.add(b);
    return bodies;
  }

  addJoint(joint) {
    this.joints.push(joint);
    return joint;
  }

  remove(body) {
    const i = this.bodies.indexOf(body);
    if (i < 0) return false;
    this.bodies.splice(i, 1);
    this.joints = this.joints.filter((j) => j.a !== body && j.b !== body && j.body !== body);
    for (const key of [...this.arbiters.keys()]) {
      const arb = this.arbiters.get(key);
      if (arb.a === body || arb.b === body) this.arbiters.delete(key);
    }
    return true;
  }

  removeJoint(joint) {
    const i = this.joints.indexOf(joint);
    if (i >= 0) this.joints.splice(i, 1);
  }

  clear() {
    // Empty the existing arrays rather than replacing them, so anything holding
    // a reference to `world.bodies` keeps seeing the live collection.
    this.bodies.length = 0;
    this.joints.length = 0;
    this.arbiters.clear();
    this.impacts.length = 0;
    this.accumulator = 0;
    this.time = 0;
    this.stepCount = 0;
    this.onPreStep = null;
  }

  on(event, handler) {
    (this.listeners[event] ||= []).push(handler);
    return () => {
      this.listeners[event] = this.listeners[event].filter((h) => h !== handler);
    };
  }

  emit(event, payload) {
    for (const handler of this.listeners[event] || []) handler(payload);
  }

  bodyAt(point, padding = 0) {
    // Last added wins, so a body dropped on top of a pile is the one grabbed.
    for (let i = this.bodies.length - 1; i >= 0; i--) {
      const body = this.bodies[i];
      if (body.containsPoint(point)) return body;
      if (padding > 0 && m.dist(body.position, point) <= body.boundingRadius + padding) return body;
    }
    return null;
  }

  raycast(origin, direction, maxDistance = Infinity, filter = null) {
    let best = null;
    for (const body of this.bodies) {
      if (filter && !filter(body)) continue;
      const hit = raycastBody(body, origin, direction, maxDistance);
      if (hit && (!best || hit.t < best.t)) best = { ...hit, body };
    }
    return best;
  }

  totalEnergy() {
    let kinetic = 0;
    let potential = 0;
    const floor = this.bounds.y + this.bounds.height;
    for (const body of this.bodies) {
      if (body.isStatic) continue;
      kinetic += body.kineticEnergy();
      potential += body.mass * m.len(this.gravity) * ((floor - body.position.y) / 1000);
    }
    return { kinetic, potential, total: kinetic + potential };
  }

  /**
   * Advance by a wall-clock delta, in seconds. Physics runs at `fixedDt`, so
   * behaviour does not change with the display's refresh rate.
   */
  update(deltaSeconds) {
    // Retire expired impacts first and unconditionally. Behind the "is anything
    // colliding?" guard below they would persist for as long as the scene stays
    // still, which is exactly when they are being looked at.
    this.retireImpacts();

    if (!Number.isFinite(deltaSeconds) || deltaSeconds <= 0) return 0;
    this.accumulator += Math.min(deltaSeconds, 0.25);

    const raw = Math.floor(this.accumulator / this.fixedDt);
    // A clamp written as Math.max(0, raw) would pass NaN straight through and
    // the loop would silently run zero times; test for finiteness explicitly.
    const steps = Number.isFinite(raw) ? Math.min(Math.max(raw, 0), this.maxSubSteps) : 0;
    for (let i = 0; i < steps; i++) this.step(this.fixedDt);

    this.accumulator -= steps * this.fixedDt;
    if (this.accumulator > this.fixedDt * this.maxSubSteps || !Number.isFinite(this.accumulator)) {
      this.accumulator = 0; // Dropped frames: give up the backlog rather than spiral.
    }
    return steps;
  }

  /** One fixed physics step. */
  step(dt) {
    const invDt = dt > 0 ? 1 / dt : 0;

    // Scene-specific forces belong to the physics step, not to the frame: at a
    // time scale of 0.1 a per-frame hook would apply a tenth of the force.
    if (this.onPreStep) this.onPreStep(dt, this);

    this.integrateForces(dt);
    this.broadPhase();

    const config = {
      gravity: this.gravity,
      dt,
      allowedPenetration: this.allowedPenetration,
      biasFactor: this.biasFactor,
      maxCorrection: this.maxCorrection,
      restitutionThreshold: this.restitutionThreshold,
      warmStarting: this.warmStarting,
    };
    for (const arb of this.arbiters.values()) arb.preStep(invDt, config);
    for (const joint of this.joints) joint.preStep(invDt);

    const iterations = Math.max(1, Math.round(this.velocityIterations) || 1);
    for (let i = 0; i < iterations; i++) {
      for (const arb of this.arbiters.values()) arb.applyImpulse();
      for (const joint of this.joints) joint.applyImpulse();
    }

    this.integrateVelocities(dt);
    this.solvePositions(config);

    // The clamp must run *after* collision resolution, not only during
    // integration: resolving a contact displaces a body along the contact
    // normal with no knowledge of the world edge the clamp just enforced.
    if (this.clampToBounds) this.applyBounds();

    this.breakOverloadedJoints(invDt);
    this.time += dt;
    this.stepCount++;
  }

  /** Overlap removal, iterated until every contact is within the slop. */
  solvePositions(config) {
    const iterations = Math.max(0, Math.round(this.positionIterations) || 0);
    for (let i = 0; i < iterations; i++) {
      let worst = 0;
      for (const arb of this.arbiters.values()) {
        worst = Math.min(worst, arb.solvePosition(config));
      }
      if (worst > -config.allowedPenetration * 1.5) break;
    }
    for (const body of this.bodies) {
      if (!body.isStatic) body.updateAABB();
    }
  }

  integrateForces(dt) {
    for (const body of this.bodies) {
      if (body.isStatic) {
        body.clearForces();
        continue;
      }
      if (!body.sanitize()) {
        body.clearForces();
        continue;
      }
      const g = m.scale(this.gravity, body.gravityScale);
      body.velocity = m.add(body.velocity, m.scale(m.add(g, m.scale(body.force, body.invMass)), dt));
      body.angularVelocity += body.torque * body.invInertia * dt;

      // Damping applied per unit time, so it does not depend on the substep rate.
      const linear = body.linearDamping ?? this.linearDamping;
      const angular = body.angularDamping ?? this.angularDamping;
      body.velocity = m.scale(body.velocity, Math.max(0, 1 - linear * dt));
      body.angularVelocity *= Math.max(0, 1 - angular * dt);

      body.clearForces();
    }
  }

  integrateVelocities(dt) {
    for (const body of this.bodies) {
      if (body.isStatic) continue;
      body.position = m.add(body.position, m.scale(body.velocity, dt));
      body.angle += body.angularVelocity * dt;
      body.sanitize();
      body.updateAABB();
    }
  }

  /** Sort-and-sweep on x, then an exact test per surviving pair. */
  broadPhase() {
    for (const body of this.bodies) body.updateAABB();

    const sorted = [...this.bodies].sort((a, b) => a.aabb.minX - b.aabb.minX);
    const active = new Set();
    let checks = 0;
    let contacts = 0;

    for (const arb of this.arbiters.values()) arb.touching = false;

    for (let i = 0; i < sorted.length; i++) {
      const body = sorted[i];
      for (const other of active) {
        if (other.aabb.maxX < body.aabb.minX) {
          active.delete(other);
          continue;
        }
        checks++;
        if (!this.shouldCollide(body, other)) continue;
        if (!aabbOverlap(body.aabb, other.aabb)) continue;

        // Canonical ordering by id keeps the manifold normal's sense stable,
        // which is what lets warm-start impulses carry over between steps.
        const [a, b] = body.id < other.id ? [body, other] : [other, body];
        const manifold = collide(a, b);
        const key = pairKey(a, b);
        if (!manifold) {
          this.arbiters.delete(key);
          continue;
        }
        let arb = this.arbiters.get(key);
        if (!arb) {
          arb = new Arbiter(a, b);
          this.arbiters.set(key, arb);
        }
        const wasTouching = arb.contacts.length > 0;
        arb.update(manifold);
        contacts += arb.contacts.length;
        if (!wasTouching) this.recordImpact(arb);
      }
      active.add(body);
    }

    for (const [key, arb] of this.arbiters) {
      if (!arb.touching) this.arbiters.delete(key);
    }

    this.stats.pairs = this.arbiters.size;
    this.stats.contacts = contacts;
    this.stats.broadPhaseChecks = checks;
  }

  shouldCollide(a, b) {
    if (a.isStatic && b.isStatic) return false;
    if (a.invMass === 0 && b.invMass === 0) return false;
    for (const joint of this.joints) {
      if (joint.collideConnected) continue;
      if ((joint.a === a && joint.b === b) || (joint.a === b && joint.b === a)) return false;
    }
    return true;
  }

  recordImpact(arb) {
    const c = arb.contacts[0];
    if (!c) return;
    const dv = relativeVelocity(arb.a, arb.b, c.r1 ?? m.sub(c.point, arb.a.position), c.r2 ?? m.sub(c.point, arb.b.position));
    const speed = Math.abs(m.dot(dv, arb.normal));
    if (speed < 60) return;
    const impact = {
      point: m.clone(c.point),
      normal: m.clone(arb.normal),
      speed,
      // Effects expire on wall-clock time, not on a frame count: a throttled or
      // hidden tab delivers no frames, and a frame-counted lifetime would never
      // run out.
      expires: this.time + 0.45,
    };
    this.impacts.push(impact);
    if (this.impacts.length > 240) this.impacts.shift();
    this.emit('collision', { a: arb.a, b: arb.b, point: impact.point, normal: impact.normal, speed });
  }

  retireImpacts() {
    let i = 0;
    while (i < this.impacts.length) {
      if (this.impacts[i].expires <= this.time) this.impacts.splice(i, 1);
      else i++;
    }
  }

  applyBounds() {
    const { x, y, width, height } = this.bounds;
    for (const body of this.bodies) {
      if (body.isStatic) continue;
      const r = body.boundingRadius;
      const minX = x + r;
      const maxX = x + width - r;
      const minY = y + r;
      const maxY = y + height - r;
      if (maxX < minX || maxY < minY) continue; // Body larger than the world.

      if (body.position.x < minX) {
        body.position.x = minX;
        // Reflect only when still moving into the wall, or a rebound already
        // applied by the contact solver is applied a second time.
        if (body.velocity.x < 0) body.velocity.x *= -body.restitution;
      } else if (body.position.x > maxX) {
        body.position.x = maxX;
        if (body.velocity.x > 0) body.velocity.x *= -body.restitution;
      }
      if (body.position.y < minY) {
        body.position.y = minY;
        if (body.velocity.y < 0) body.velocity.y *= -body.restitution;
      } else if (body.position.y > maxY) {
        body.position.y = maxY;
        if (body.velocity.y > 0) body.velocity.y *= -body.restitution;
      }
      body.updateAABB();
    }
  }

  breakOverloadedJoints(invDt) {
    if (!this.joints.length) return;
    const survivors = [];
    for (const joint of this.joints) {
      if (joint.maxForce !== undefined && Number.isFinite(joint.maxForce) && joint.reactionForce) {
        if (joint.reactionForce(invDt) > joint.maxForce) {
          joint.broken = true;
          continue;
        }
      }
      survivors.push(joint);
    }
    if (survivors.length !== this.joints.length) this.joints = survivors;
  }

  /** Grab a body with a soft mouse constraint. Returns the joint, or null. */
  grab(point, options = {}) {
    const body = this.bodyAt(point, options.padding ?? 0);
    if (!body || body.isStatic) return null;
    const joint = new MouseJoint(body, point, options);
    this.joints.push(joint);
    return joint;
  }

  release(joint) {
    if (joint) this.removeJoint(joint);
  }

  /** Radial impulse, falling off linearly to zero at the radius. */
  explode(center, radius, strength) {
    for (const body of this.bodies) {
      if (body.isStatic) continue;
      const d = m.sub(body.position, center);
      const distance = m.len(d);
      if (distance > radius) continue;
      const falloff = 1 - distance / radius;
      const dir = distance > m.EPS ? m.scale(d, 1 / distance) : m.v(0, -1);
      body.applyImpulse(m.scale(dir, strength * falloff * body.mass), body.position);
    }
  }

  setBounds(bounds) {
    this.bounds = { ...bounds };
  }
}
