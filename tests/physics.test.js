import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as m from '../src/math.js';
import { World } from '../src/world.js';
import { box, circle, ngon } from '../src/body.js';
import { DistanceJoint, RevoluteJoint } from '../src/joints.js';
import { collide, raycastBody } from '../src/collide.js';

const G = 900;

function makeWorld(options = {}) {
  return new World({
    gravity: m.v(0, G),
    bounds: { x: 0, y: 0, width: 1200, height: 800 },
    ...options,
  });
}

function ground(world, { friction = 0.6, y = 780 } = {}) {
  return world.add(box(600, y, 640, 20, { isStatic: true, friction, restitution: 0 }));
}

function run(world, seconds, dt = world.fixedDt) {
  const steps = Math.round(seconds / dt);
  for (let i = 0; i < steps; i++) world.step(dt);
  return steps;
}

test('a falling box comes to rest on the ground without sinking through it', () => {
  const world = makeWorld();
  ground(world);
  const b = world.add(box(600, 300, 30, 30, { friction: 0.5, restitution: 0 }));

  run(world, 3);

  const restY = 760 - 30; // ground top minus the box half-height
  assert.ok(Math.abs(b.position.y - restY) < 2, `settled at ${b.position.y.toFixed(2)}, expected ~${restY}`);
  assert.ok(Math.abs(b.velocity.y) < 5, 'at rest vertically');
});

/** Highest point reached *after* the body first starts moving upward. */
function reboundPeak(world, body, steps = 500) {
  let landed = false;
  let peak = Infinity;
  for (let i = 0; i < steps; i++) {
    world.step(world.fixedDt);
    if (!landed && body.velocity.y < 0) landed = true;
    if (landed) peak = Math.min(peak, body.position.y);
  }
  return peak;
}

test('restitution 0 does not bounce and restitution 0.9 does', () => {
  const dead = makeWorld();
  ground(dead, { friction: 0.9 });
  const a = dead.add(circle(600, 300, 25, { restitution: 0, friction: 0.9 }));
  const peakA = reboundPeak(dead, a);
  assert.ok(peakA > 720, `dead ball should not rebound, rose to ${peakA.toFixed(1)}`);

  const live = makeWorld();
  ground(live, { friction: 0.9 });
  const b = live.add(circle(600, 300, 25, { restitution: 0.9, friction: 0.9 }));
  const peakB = reboundPeak(live, b);
  assert.ok(peakB < 450, `bouncy ball should rebound, only rose to ${peakB.toFixed(1)}`);
});

test('a perfectly elastic ball loses a little height per bounce and never gains', () => {
  // A discrete step overshoots the surface, so the naive reflection returns
  // more speed than went in. The solver discounts the gravity picked up while
  // the body was below the surface, which puts the error on the safe side.
  const world = makeWorld({ linearDamping: 0 });
  ground(world);
  const b = world.add(circle(600, 200, 24, { restitution: 1, friction: 0 }));
  const start = b.position.y;
  let highest = start;
  let bounces = 0;
  let rising = false;
  for (let i = 0; i < 2000; i++) {
    world.step(world.fixedDt);
    if (b.velocity.y < 0 && !rising) {
      rising = true;
      bounces++;
    } else if (b.velocity.y >= 0) {
      rising = false;
    }
    highest = Math.min(highest, b.position.y);
  }
  assert.ok(bounces >= 5, `expected repeated bounces, saw ${bounces}`);
  assert.ok(highest >= start, `ball rose above its drop height: ${highest.toFixed(1)} vs ${start}`);
  assert.ok(highest < start + 60, `ball is over-damped, only reached ${highest.toFixed(1)}`);
});

test('a ball rolls down the shallowest slope instead of stalling', () => {
  // Coulomb friction is bounded by the normal impulse. Scaling tangential
  // velocity by a flat coefficient instead would compound at the substep rate
  // and stop the ball dead on a gentle ramp.
  const world = makeWorld();
  // Canvas y grows downward, so a negative angle tilts the ramp down to the left.
  const slope = -0.18; // about 10 degrees
  world.add(box(600, 600, 500, 12, { isStatic: true, angle: slope, friction: 0.6 }));
  const ball = world.add(circle(600, 520, 20, { friction: 0.6, restitution: 0 }));

  run(world, 0.4); // let it settle onto the ramp
  const startX = ball.position.x;
  run(world, 1.6);
  const travelled = ball.position.x - startX;

  assert.ok(travelled < -150, `ball only travelled ${travelled.toFixed(1)}px down a 10 degree ramp`);
  assert.ok(ball.angularVelocity < -0.5, 'ball is rolling, not sliding');
});

test('friction still holds a box on a slope below the friction angle', () => {
  const world = makeWorld();
  world.add(box(600, 600, 500, 12, { isStatic: true, angle: -0.12, friction: 0.9 }));
  const b = world.add(box(600, 540, 30, 20, { friction: 0.9, angle: -0.12, restitution: 0 }));

  run(world, 0.5);
  const startX = b.position.x;
  run(world, 2.5);
  assert.ok(Math.abs(b.position.x - startX) < 25, `box slid ${(b.position.x - startX).toFixed(1)}px when it should grip`);
});

test('a stack of boxes stays stacked and stays put', () => {
  const world = makeWorld();
  ground(world);
  const stack = [];
  for (let i = 0; i < 8; i++) {
    stack.push(world.add(box(600, 740 - i * 40.5, 20, 20, { friction: 0.6, restitution: 0 })));
  }
  run(world, 6);

  for (let i = 0; i < stack.length; i++) {
    assert.ok(Math.abs(stack[i].position.x - 600) < 12, `box ${i} drifted to x=${stack[i].position.x.toFixed(1)}`);
  }
  const heights = stack.map((b) => b.position.y);
  for (let i = 1; i < heights.length; i++) {
    assert.ok(heights[i] < heights[i - 1], 'stack order preserved');
  }
  const fastest = Math.max(...stack.map((b) => m.len(b.velocity)));
  assert.ok(fastest < 3, `stack is still jittering at ${fastest.toFixed(2)}px/s`);
});

test('bodies stay inside the world bounds over a long run', () => {
  // The clamp runs after collision resolution, so a body squeezed against a
  // wall by another body cannot be pushed through it.
  const world = makeWorld();
  ground(world);
  const rng = m.mulberry32(12345);
  for (let i = 0; i < 30; i++) {
    const x = 40 + rng() * 1120;
    const y = 60 + rng() * 500;
    world.add(rng() > 0.5 ? circle(x, y, 12 + rng() * 14) : box(x, y, 12 + rng() * 12, 12 + rng() * 12));
  }
  // Squeeze everything into the left corner.
  world.gravity = m.v(-1400, 900);

  for (let i = 0; i < 2400; i++) {
    world.step(world.fixedDt);
    for (const b of world.bodies) {
      if (b.isStatic) continue;
      const r = b.boundingRadius;
      assert.ok(
        b.position.x >= -r - 1 && b.position.x <= 1200 + r + 1 && b.position.y >= -r - 1 && b.position.y <= 800 + r + 1,
        `body escaped at step ${i}: (${b.position.x.toFixed(1)}, ${b.position.y.toFixed(1)})`,
      );
    }
  }
});

test('a non-finite body is parked rather than poisoning the step', () => {
  const world = makeWorld();
  ground(world);
  const good = world.add(circle(500, 200, 20));
  const bad = world.add(circle(700, 200, 20));
  bad.velocity = m.v(NaN, 0);

  const steps = world.update(1 / 60);
  assert.ok(steps > 0, 'the step loop still ran; a NaN-derived count would silently run zero times');
  assert.ok(Number.isFinite(bad.position.x) && Number.isFinite(bad.position.y));
  assert.ok(Number.isFinite(good.position.y) && good.position.y > 200, 'the healthy body still fell');
});

test('impacts are retired while the world is idle', () => {
  // The sweep runs before the "is anything happening?" guard, so effects do not
  // outlive their lifetime just because the scene has gone quiet.
  const world = makeWorld();
  ground(world);
  world.add(circle(600, 300, 25, { restitution: 0.1 }));
  for (let i = 0; i < 200; i++) world.step(world.fixedDt);
  assert.ok(world.impacts.length > 0, 'the landing registered an impact');

  for (let i = 0; i < 200; i++) world.step(world.fixedDt);
  world.update(1 / 60);
  assert.equal(world.impacts.length, 0, 'stale impacts survived into the idle period');
});

test('update ignores a non-finite or negative delta', () => {
  const world = makeWorld();
  const b = world.add(circle(600, 300, 20));
  const y = b.position.y;
  assert.equal(world.update(NaN), 0);
  assert.equal(world.update(-1), 0);
  assert.equal(b.position.y, y);
});

test('physics is frame-rate independent', () => {
  const fall = (frameDt, frames) => {
    const world = makeWorld({ linearDamping: 0 });
    const b = world.add(circle(600, 100, 15, { restitution: 0 }));
    for (let i = 0; i < frames; i++) world.update(frameDt);
    return b.position.y;
  };
  const at60 = fall(1 / 60, 60);
  const at30 = fall(1 / 30, 30);
  assert.ok(Math.abs(at60 - at30) < 2, `60Hz gave ${at60.toFixed(2)}, 30Hz gave ${at30.toFixed(2)}`);
});

test('a distance joint holds its length', () => {
  const world = makeWorld();
  const anchor = world.add(circle(600, 100, 8, { isStatic: true }));
  const bob = world.add(circle(800, 100, 20));
  world.addJoint(new DistanceJoint(anchor, bob, anchor.position, bob.position, { stiffness: 1 }));

  run(world, 4);
  const length = m.dist(anchor.position, bob.position);
  assert.ok(Math.abs(length - 200) < 12, `pendulum length drifted to ${length.toFixed(1)}`);
});

test('a rope pulls but never pushes', () => {
  const world = makeWorld({ gravity: m.v(0, 0) });
  const a = world.add(circle(600, 400, 10, { isStatic: true }));
  const b = world.add(circle(700, 400, 10));
  world.addJoint(new DistanceJoint(a, b, a.position, b.position, { rope: true, length: 100 }));

  // Drive b towards a: a rope must not resist.
  b.velocity = m.v(-200, 0);
  run(world, 0.2);
  assert.ok(m.dist(a.position, b.position) < 80, 'rope wrongly resisted compression');

  // Drive b away: the rope must catch it.
  b.velocity = m.v(600, 0);
  run(world, 1.5);
  assert.ok(m.dist(a.position, b.position) < 115, 'rope let the body stretch past its length');
});

test('a revolute joint keeps its anchors coincident', () => {
  const world = makeWorld();
  const a = world.add(box(600, 120, 40, 10, { isStatic: true }));
  const b = world.add(box(700, 120, 60, 10));
  world.addJoint(new RevoluteJoint(a, b, m.v(640, 120)));

  run(world, 3);
  const [p1, p2] = world.joints[0].anchors();
  assert.ok(m.dist(p1, p2) < 6, `pin drifted apart by ${m.dist(p1, p2).toFixed(2)}px`);
});

test('collide reports a normal pointing from a towards b', () => {
  const a = circle(0, 0, 10);
  const b = circle(15, 0, 10);
  const manifold = collide(a, b);
  assert.ok(manifold);
  assert.ok(manifold.normal.x > 0.99, 'normal points a -> b');
  assert.ok(Math.abs(manifold.contacts[0].penetration - 5) < 1e-9);
});

test('a resting box on a box reports two coplanar contacts', () => {
  const a = box(0, 0, 50, 10, { isStatic: true });
  const b = box(0, -19.5, 20, 10);
  const manifold = collide(a, b);
  assert.ok(manifold);
  assert.equal(manifold.contacts.length, 2, 'face-to-face contact needs two points to resist rocking');
});

test('separated shapes report no manifold', () => {
  assert.equal(collide(circle(0, 0, 10), circle(100, 0, 10)), null);
  assert.equal(collide(box(0, 0, 10, 10), box(100, 0, 10, 10)), null);
  assert.equal(collide(box(0, 0, 10, 10), circle(100, 0, 10)), null);
});

test('raycast hits a box face and a circle at the expected distance', () => {
  const b = box(100, 0, 20, 20);
  const hit = raycastBody(b, m.v(0, 0), m.v(1, 0));
  assert.ok(hit && Math.abs(hit.t - 80) < 1e-6, `expected t=80, got ${hit && hit.t}`);
  assert.ok(hit.normal.x < -0.99);

  const c = circle(100, 0, 20);
  const chit = raycastBody(c, m.v(0, 0), m.v(1, 0));
  assert.ok(chit && Math.abs(chit.t - 80) < 1e-6);
  assert.equal(raycastBody(c, m.v(0, 500), m.v(1, 0)), null);
});

test('world.raycast returns the nearest body', () => {
  const world = makeWorld();
  world.add(box(300, 400, 20, 20));
  const near = world.add(box(200, 400, 20, 20));
  const hit = world.raycast(m.v(0, 400), m.v(1, 0));
  assert.equal(hit.body, near);
});

test('removing a body drops its joints and contacts', () => {
  const world = makeWorld();
  ground(world);
  const a = world.add(circle(600, 700, 20));
  const b = world.add(circle(600, 640, 20));
  world.addJoint(new DistanceJoint(a, b, a.position, b.position));
  run(world, 0.5);

  world.remove(a);
  assert.equal(world.joints.length, 0);
  for (const arb of world.arbiters.values()) {
    assert.ok(arb.a !== a && arb.b !== a, 'a stale arbiter kept a removed body alive');
  }
  run(world, 0.5); // must not throw
});

test('clear() keeps the same array identity so held references stay live', () => {
  const world = makeWorld();
  const bodies = world.bodies;
  world.add(circle(0, 0, 5));
  world.clear();
  assert.equal(world.bodies, bodies, 'clear replaced the array a caller may already hold');
  assert.equal(bodies.length, 0);
});

test('an explosion pushes bodies away from its centre and leaves distant ones alone', () => {
  const world = makeWorld({ gravity: m.v(0, 0) });
  const near = world.add(circle(620, 400, 15));
  const far = world.add(circle(1100, 400, 15));
  world.explode(m.v(600, 400), 200, 300);
  assert.ok(near.velocity.x > 0, 'near body pushed outward');
  assert.equal(far.velocity.x, 0, 'far body untouched');
});

test('mixed shapes settle into a pile without exploding', () => {
  const world = makeWorld();
  ground(world);
  world.add(box(300, 400, 20, 300, { isStatic: true }));
  world.add(box(900, 400, 20, 300, { isStatic: true }));
  const rng = m.mulberry32(99);
  for (let i = 0; i < 40; i++) {
    const x = 340 + rng() * 520;
    const y = 120 + rng() * 400;
    const kind = Math.floor(rng() * 3);
    if (kind === 0) world.add(circle(x, y, 10 + rng() * 10));
    else if (kind === 1) world.add(box(x, y, 10 + rng() * 10, 10 + rng() * 10));
    else world.add(ngon(x, y, 3 + Math.floor(rng() * 4), 12 + rng() * 10));
  }
  run(world, 8);

  for (const b of world.bodies) {
    assert.ok(Number.isFinite(b.position.x) && Number.isFinite(b.position.y), 'position stayed finite');
    assert.ok(m.len(b.velocity) < 400, `body is still flying at ${m.len(b.velocity).toFixed(0)}px/s after settling`);
  }
  const fastest = Math.max(...world.bodies.filter((b) => !b.isStatic).map((b) => m.len(b.velocity)));
  assert.ok(fastest < 25, `pile has not settled, fastest body is at ${fastest.toFixed(1)}px/s`);
});
