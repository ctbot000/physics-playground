import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as m from '../src/math.js';
import { World } from '../src/world.js';
import { scenes, sceneById, addBounds, resizeBounds, floorTop, buildChain, blobBody } from '../src/scenes.js';
import { box, circle } from '../src/body.js';
import { DistanceJoint } from '../src/joints.js';
import { collide } from '../src/collide.js';

const SIZES = [
  [1200, 760],
  [560, 420],
  [380, 680],
  [1800, 950],
];

function boot(scene, width, height, seed = 7) {
  const world = new World({
    gravity: scene.gravity ? m.clone(scene.gravity) : m.v(0, 900),
    bounds: { x: 0, y: 0, width, height },
    ...(scene.tune || {}),
  });
  if (scene.update) world.onPreStep = (dt) => scene.update(world, dt);
  scene.build(world, { width, height, rng: m.mulberry32(seed) });
  return world;
}

function run(world, seconds) {
  const steps = Math.round(seconds / world.fixedDt);
  for (let i = 0; i < steps; i++) world.step(world.fixedDt);
}

test('every scene has a unique id, a name and a hint', () => {
  const ids = new Set();
  for (const scene of scenes) {
    assert.ok(scene.id && !ids.has(scene.id), `duplicate or missing id: ${scene.id}`);
    ids.add(scene.id);
    assert.ok(scene.name && scene.hint, `${scene.id} is missing a name or hint`);
    assert.equal(typeof scene.build, 'function');
  }
  assert.equal(sceneById('nope').id, scenes[0].id, 'an unknown id falls back to the first scene');
});

for (const scene of scenes) {
  test(`scene "${scene.id}" settles at every viewport size`, () => {
    for (const [width, height] of SIZES) {
      const world = boot(scene, width, height);
      run(world, 8);

      for (const body of world.bodies) {
        assert.ok(
          Number.isFinite(body.position.x) && Number.isFinite(body.position.y) && Number.isFinite(body.angle),
          `${scene.id} at ${width}x${height}: body ${body.id} went non-finite`,
        );
        assert.ok(m.len(body.velocity) < 4000, `${scene.id} at ${width}x${height}: body ${body.id} is at ${m.len(body.velocity).toFixed(0)}px/s`);
      }
    }
  });
}

test('scenes that build joints do not shed them on their own', () => {
  for (const scene of scenes) {
    const world = boot(scene, 1200, 760);
    const before = world.joints.length;
    if (before === 0) continue;
    run(world, 8);
    assert.equal(world.joints.length, before, `${scene.id} broke ${before - world.joints.length} of its own joints`);
  }
});

test('the floor helper reports the surface bodies actually rest on', () => {
  const world = new World({ gravity: m.v(0, 900), bounds: { x: 0, y: 0, width: 800, height: 600 } });
  addBounds(world, { width: 800, height: 600 });
  const b = world.add(box(400, 200, 20, 20, { friction: 0.6, restitution: 0 }));
  run(world, 3);
  assert.ok(
    Math.abs(b.position.y - (floorTop(600) - 20)) < 2,
    `rested at ${b.position.y.toFixed(1)}, floorTop says ${floorTop(600)}`,
  );
});

test('a small body cannot roll out under something standing on the floor', () => {
  // Anything built to floorTop() sits flush; a wall stopping short leaves a gap
  // that a ball rolls straight through, which reads as a collision bug.
  const world = new World({ gravity: m.v(0, 900), bounds: { x: 0, y: 0, width: 800, height: 600 } });
  addBounds(world, { width: 800, height: 600 });
  const surface = floorTop(600);
  for (const x of [300, 500]) {
    world.add(box(x, surface - 60, 3, 60, { isStatic: true, friction: 0.3 }));
  }
  const ball = world.add(circle(400, surface - 100, 8, { friction: 0.2, restitution: 0.1 }));
  ball.velocity = m.v(-900, 0);
  run(world, 4);
  assert.ok(ball.position.x > 296 && ball.position.x < 504, `ball escaped the pen at x=${ball.position.x.toFixed(1)}`);
});

test('resizing the bounds keeps a settled scene inside them', () => {
  const scene = sceneById('pyramid');
  const world = boot(scene, 1200, 760);
  run(world, 3);
  world.setBounds({ x: 0, y: 0, width: 620, height: 900 });
  resizeBounds(world, 620, 900);
  run(world, 5);
  for (const body of world.bodies) {
    if (body.isStatic) continue;
    assert.ok(body.position.y < 900, `body fell past the moved floor: y=${body.position.y.toFixed(1)}`);
  }
});

test('a dense network of rigid distance joints stays bounded', () => {
  // Solved with a plain Baumgarte bias this diverges: constraint force reaches
  // thousands of times the mesh's own weight, and raising the iteration count
  // makes it worse rather than better.
  const world = new World({
    gravity: m.v(0, 900),
    bounds: { x: 0, y: 0, width: 1200, height: 760 },
    velocityIterations: 16,
  });
  const cols = 14;
  const rows = 10;
  const spacing = 34;
  const grid = [];
  for (let row = 0; row < rows; row++) {
    grid[row] = [];
    for (let col = 0; col < cols; col++) {
      const pinned = row === 0 && (col === 0 || col === cols - 1);
      grid[row][col] = world.add(circle(300 + col * spacing, 120 + row * spacing, 5, {
        isStatic: pinned, density: 0.9,
      }));
    }
  }
  const link = (p, q) => world.addJoint(new DistanceJoint(p, q, p.position, q.position, { stiffness: 1, damping: 0.4 }));
  for (let row = 0; row < rows; row++) {
    for (let col = 0; col < cols; col++) {
      if (col + 1 < cols) link(grid[row][col], grid[row][col + 1]);
      if (row + 1 < rows) link(grid[row][col], grid[row + 1][col]);
    }
  }

  const nodeWeight = grid[1][0].mass * 900;
  let peak = 0;
  for (let i = 0; i < 960; i++) {
    world.step(world.fixedDt);
    if (i > 600) for (const joint of world.joints) peak = Math.max(peak, joint.reactionForce(1 / world.fixedDt));
  }
  assert.ok(peak / nodeWeight < 1000, `constraint force reached ${(peak / nodeWeight).toFixed(0)}x a node's weight`);
  const fastest = Math.max(...world.bodies.filter((b) => !b.isStatic).map((b) => m.len(b.velocity)));
  assert.ok(fastest < 150, `mesh has not settled: fastest node at ${fastest.toFixed(0)}px/s`);
});

test('a rigid rod still holds its length despite being solved as a stiff spring', () => {
  const world = new World({ gravity: m.v(0, 900), bounds: { x: 0, y: 0, width: 1200, height: 800 } });
  const a = world.add(circle(600, 100, 8, { isStatic: true }));
  const b = world.add(circle(800, 100, 20, { density: 4 }));
  world.addJoint(new DistanceJoint(a, b, a.position, b.position, { stiffness: 1 }));
  run(world, 4);
  const length = m.dist(a.position, b.position);
  assert.ok(Math.abs(length - 200) < 3, `rod stretched to ${length.toFixed(2)}`);
});

test('contact ids within one manifold are distinct', () => {
  // Two contacts sharing an id warm-start from the same stored impulse, which
  // loads one side of a resting box harder than the other and tips a stack.
  const a = box(0, 0, 60, 10, { isStatic: true });
  for (const angle of [0, 0.01, -0.02]) {
    const b = box(5, -19.4, 20, 10, { angle });
    const manifold = collide(a, b);
    assert.ok(manifold, `no manifold at angle ${angle}`);
    const ids = new Set(manifold.contacts.map((c) => c.id));
    assert.equal(ids.size, manifold.contacts.length, `duplicate contact id at angle ${angle}`);
  }
});

test('buildChain hangs a chain from a fixed anchor', () => {
  const world = new World({ gravity: m.v(0, 900), bounds: { x: 0, y: 0, width: 1200, height: 760 } });
  const created = buildChain(world, m.v(600, 100), m.v(800, 100), 8);
  assert.equal(created.length, 9);
  assert.ok(created[0].isStatic, 'the first link is the anchor');
  run(world, 5);
  assert.ok(created[8].position.y > 200, 'the free end hangs down');
  assert.ok(m.dist(created[0].position, m.v(600, 100)) < 1e-6, 'the anchor did not move');
});

test('blobBody makes a convex polygon with a sane mass', () => {
  const rng = m.mulberry32(4);
  for (let i = 0; i < 20; i++) {
    const body = blobBody(0, 0, 30, rng, { density: 1 });
    assert.ok(body.shape.vertices.length >= 3);
    assert.ok(body.mass > 0 && Number.isFinite(body.inertia));
    assert.ok(body.containsPoint(m.v(0, 0)), 'the centroid is inside the blob');
  }
});

test('an untouched scene rebuilt at a new size fits the new size', () => {
  for (const scene of scenes) {
    const world = boot(scene, 400, 320);
    for (const body of world.bodies) {
      assert.ok(
        body.aabb.minX > -400 && body.aabb.maxX < 800 && body.aabb.maxY < 720,
        `${scene.id} placed a body outside a 400x320 world: ${JSON.stringify(body.aabb)}`,
      );
    }
  }
});
