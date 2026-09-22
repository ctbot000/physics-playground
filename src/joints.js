// Constraints between bodies. Each joint exposes preStep(invDt) and
// applyImpulse(), matching the contact solver so they can be iterated together.

import * as m from './math.js';

let nextJointId = 1;

/**
 * Constraint frequency used for a "rigid" distance joint, capped at a third of
 * the step rate so it stays well inside what the fixed step can represent.
 */
const RIGID_FREQUENCY = 40;

class Joint {
  constructor(a, b) {
    this.id = nextJointId++;
    this.a = a;
    this.b = b;
    this.broken = false;
  }

  /** Both endpoints in world space, for rendering and for break checks. */
  anchors() {
    return [this.a.position, this.b.position];
  }
}

/**
 * Pin joint: holds two local anchor points coincident. Two scalar constraints,
 * solved as one 2x2 block so the pair converges in a single iteration.
 */
export class RevoluteJoint extends Joint {
  constructor(a, b, worldAnchor, options = {}) {
    super(a, b);
    this.localA = a.toLocal(worldAnchor);
    this.localB = b.toLocal(worldAnchor);
    this.impulse = m.v(0, 0);
    this.softness = options.softness ?? 0;
    this.biasFactor = options.biasFactor ?? 0.2;
    this.maxForce = options.maxForce ?? Infinity;
    this.collideConnected = options.collideConnected ?? false;
    this.kind = 'revolute';
  }

  anchors() {
    return [this.a.toWorld(this.localA), this.b.toWorld(this.localB)];
  }

  preStep(invDt) {
    const { a, b } = this;
    this.r1 = m.rotate(this.localA, a.angle);
    this.r2 = m.rotate(this.localB, b.angle);

    const k11 = a.invMass + b.invMass + a.invInertia * this.r1.y ** 2 + b.invInertia * this.r2.y ** 2;
    const k12 = -a.invInertia * this.r1.x * this.r1.y - b.invInertia * this.r2.x * this.r2.y;
    const k22 = a.invMass + b.invMass + a.invInertia * this.r1.x ** 2 + b.invInertia * this.r2.x ** 2;
    this.mass = m.invert2x2(k11 + this.softness, k12, k22 + this.softness);

    const p1 = m.add(a.position, this.r1);
    const p2 = m.add(b.position, this.r2);
    const separation = m.sub(p2, p1);
    this.bias = m.scale(separation, -this.biasFactor * invDt);

    // Warm start.
    a.applyImpulse(m.neg(this.impulse), p1);
    b.applyImpulse(this.impulse, p2);
  }

  applyImpulse() {
    if (!this.mass) return;
    const { a, b } = this;
    const dv = m.sub(
      m.add(b.velocity, m.crossSV(b.angularVelocity, this.r2)),
      m.add(a.velocity, m.crossSV(a.angularVelocity, this.r1)),
    );
    const rhsX = this.bias.x - dv.x - this.softness * this.impulse.x;
    const rhsY = this.bias.y - dv.y - this.softness * this.impulse.y;
    const impulse = {
      x: this.mass.a * rhsX + this.mass.b * rhsY,
      y: this.mass.b * rhsX + this.mass.d * rhsY,
    };
    this.impulse = m.add(this.impulse, impulse);

    a.applyImpulse(m.neg(impulse), m.add(a.position, this.r1));
    b.applyImpulse(impulse, m.add(b.position, this.r2));
  }

  /** Force currently carried, used for breakable joints. */
  reactionForce(invDt) {
    return m.len(this.impulse) * invDt;
  }
}

/**
 * Distance joint. With `stiffness` 1 it is a rigid rod; below that it becomes a
 * spring, expressed as a constraint with softness rather than as an explicit
 * force -- an explicit spring integrated at a fixed step goes unstable as soon
 * as the stiffness passes `2 * I_reduced / dt`, and in a chain of them the
 * reduced inertia is small enough that ordinary values cross it.
 *
 * With `rope` set it only resists stretching, never compression.
 */
export class DistanceJoint extends Joint {
  constructor(a, b, worldAnchorA, worldAnchorB, options = {}) {
    super(a, b);
    this.localA = a.toLocal(worldAnchorA);
    this.localB = b.toLocal(worldAnchorB);
    this.length = Math.max(options.length ?? m.dist(worldAnchorA, worldAnchorB), 1e-3);
    this.stiffness = m.clamp(options.stiffness ?? 1, 0, 1);
    this.dampingRatio = m.clamp(options.damping ?? 0.2, 0, 1);
    this.rope = options.rope ?? false;
    this.impulse = 0;
    this.maxForce = options.maxForce ?? Infinity;
    this.collideConnected = options.collideConnected ?? false;
    this.kind = options.kind ?? (this.stiffness >= 1 ? 'rod' : 'spring');
  }

  anchors() {
    return [this.a.toWorld(this.localA), this.b.toWorld(this.localB)];
  }

  preStep(invDt) {
    const { a, b } = this;
    const dt = 1 / invDt;
    this.r1 = m.rotate(this.localA, a.angle);
    this.r2 = m.rotate(this.localB, b.angle);
    const p1 = m.add(a.position, this.r1);
    const p2 = m.add(b.position, this.r2);
    const d = m.sub(p2, p1);
    const currentLength = m.len(d);
    this.u = currentLength > m.EPS ? m.scale(d, 1 / currentLength) : m.v(0, 1);
    this.slack = this.rope && currentLength < this.length;

    const crossA = m.cross(this.r1, this.u);
    const crossB = m.cross(this.r2, this.u);
    let invMassSum =
      a.invMass + b.invMass + a.invInertia * crossA * crossA + b.invInertia * crossB * crossB;
    if (invMassSum <= m.EPS) {
      this.effectiveMass = 0;
      this.gamma = 0;
      this.bias = 0;
      return;
    }

    const error = currentLength - this.length;

    // Both the rod and the spring are solved as the same implicit soft
    // constraint; "rigid" is just a very high frequency. Solving the rod with
    // a plain Baumgarte bias instead (gamma = 0) is stable on its own and in a
    // chain, and diverges in a network with loops -- a cloth grid built that
    // way reaches a thousand times its own weight in constraint force, and
    // adding solver iterations makes it worse rather than better.
    const reducedMass = 1 / invMassSum;
    const frequency = this.stiffness >= 1
      ? Math.min(RIGID_FREQUENCY, 1 / (3 * dt))
      : 1.5 + this.stiffness * 14;
    const dampingRatio = this.stiffness >= 1 ? Math.max(this.dampingRatio, 0.7) : this.dampingRatio;
    const omega = 2 * Math.PI * frequency;
    const damping = 2 * reducedMass * dampingRatio * omega;
    const spring = reducedMass * omega * omega;
    const denominator = dt * (damping + dt * spring);
    this.gamma = denominator > m.EPS ? 1 / denominator : 0;
    this.bias = error * dt * spring * this.gamma;
    invMassSum += this.gamma;

    this.effectiveMass = 1 / invMassSum;

    if (!this.slack) {
      const p = m.scale(this.u, this.impulse);
      a.applyImpulse(m.neg(p), p1);
      b.applyImpulse(p, p2);
    } else {
      this.impulse = 0;
    }
  }

  applyImpulse() {
    if (this.slack || !this.effectiveMass) return;
    const { a, b } = this;
    const p1 = m.add(a.position, this.r1);
    const p2 = m.add(b.position, this.r2);
    const dv = m.sub(
      m.add(b.velocity, m.crossSV(b.angularVelocity, this.r2)),
      m.add(a.velocity, m.crossSV(a.angularVelocity, this.r1)),
    );
    const vn = m.dot(dv, this.u);
    let impulse = -this.effectiveMass * (vn + this.bias + this.gamma * this.impulse);
    const old = this.impulse;
    this.impulse += impulse;
    // A rope may only pull the bodies together, never push them apart.
    if (this.rope && this.impulse > 0) this.impulse = 0;
    impulse = this.impulse - old;

    const p = m.scale(this.u, impulse);
    a.applyImpulse(m.neg(p), p1);
    b.applyImpulse(p, p2);
  }

  reactionForce(invDt) {
    return Math.abs(this.impulse) * invDt;
  }
}

/**
 * Soft point-to-target constraint used for mouse dragging. The force cap is
 * what stops a grabbed body driving through a wall it is pressed against.
 */
export class MouseJoint {
  constructor(body, worldPoint, options = {}) {
    this.id = nextJointId++;
    this.body = body;
    this.local = body.toLocal(worldPoint);
    this.target = m.clone(worldPoint);
    // Strong enough that grabbing a heavy body feels direct, and still
    // bounded so a dragged body cannot bulldoze through a wall.
    this.maxForce = options.maxForce ?? 6000 * Math.max(body.mass, 1);
    this.frequency = options.frequency ?? 7;
    this.dampingRatio = options.dampingRatio ?? 0.9;
    this.impulse = m.v(0, 0);
    this.kind = 'mouse';
    this.broken = false;
  }

  setTarget(worldPoint) {
    this.target = m.clone(worldPoint);
  }

  anchors() {
    return [this.body.toWorld(this.local), this.target];
  }

  preStep(invDt) {
    const body = this.body;
    const dt = 1 / invDt;
    this.r = m.rotate(this.local, body.angle);

    const omega = 2 * Math.PI * this.frequency;
    const mass = Math.max(body.mass, m.EPS);
    const k = mass * omega * omega;
    const c = 2 * mass * this.dampingRatio * omega;
    this.gamma = dt * (c + dt * k) > m.EPS ? 1 / (dt * (c + dt * k)) : 0;
    const beta = dt * k * this.gamma;

    const p = m.add(body.position, this.r);
    this.bias = m.scale(m.sub(p, this.target), beta * invDt);

    const k11 = body.invMass + body.invInertia * this.r.y ** 2 + this.gamma;
    const k12 = -body.invInertia * this.r.x * this.r.y;
    const k22 = body.invMass + body.invInertia * this.r.x ** 2 + this.gamma;
    this.mass = m.invert2x2(k11, k12, k22);
    this.maxImpulse = this.maxForce * dt;

    body.applyImpulse(this.impulse, p);
  }

  applyImpulse() {
    if (!this.mass) return;
    const body = this.body;
    const p = m.add(body.position, this.r);
    const dv = m.add(body.velocity, m.crossSV(body.angularVelocity, this.r));
    const rhsX = -(dv.x + this.bias.x + this.gamma * this.impulse.x);
    const rhsY = -(dv.y + this.bias.y + this.gamma * this.impulse.y);
    let impulse = {
      x: this.mass.a * rhsX + this.mass.b * rhsY,
      y: this.mass.b * rhsX + this.mass.d * rhsY,
    };

    const old = m.clone(this.impulse);
    this.impulse = m.add(this.impulse, impulse);
    const magnitude = m.len(this.impulse);
    if (magnitude > this.maxImpulse && magnitude > m.EPS) {
      this.impulse = m.scale(this.impulse, this.maxImpulse / magnitude);
    }
    impulse = m.sub(this.impulse, old);
    body.applyImpulse(impulse, p);
  }

  reactionForce(invDt) {
    return m.len(this.impulse) * invDt;
  }
}
