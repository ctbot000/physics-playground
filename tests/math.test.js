import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as m from '../src/math.js';
import { boxShape, polygonShape, convexHull, polygonArea, polygonCentroid, box, circle } from '../src/body.js';

test('cross products follow the right-hand rule', () => {
  assert.equal(m.cross(m.v(1, 0), m.v(0, 1)), 1);
  assert.deepEqual(m.crossSV(1, m.v(1, 0)), { x: -0, y: 1 });
  assert.deepEqual(m.crossVS(m.v(1, 0), 1), { x: 0, y: -1 });
});

test('rotate and unrotate are inverses', () => {
  const p = m.v(3, -7);
  const back = m.unrotate(m.rotate(p, 0.9), 0.9);
  assert.ok(m.dist(p, back) < 1e-12);
});

test('normalize of a zero vector is zero, not NaN', () => {
  const n = m.normalize(m.v(0, 0));
  assert.ok(Number.isFinite(n.x) && Number.isFinite(n.y));
});

test('invert2x2 refuses a singular block instead of returning Infinity', () => {
  assert.equal(m.invert2x2(1, 1, 1), null);
  const inv = m.invert2x2(2, 0, 4);
  assert.ok(Math.abs(inv.a - 0.5) < 1e-12 && Math.abs(inv.d - 0.25) < 1e-12);
});

test('convex hull drops interior points and winds counter-clockwise', () => {
  const hull = convexHull([
    { x: 0, y: 0 }, { x: 10, y: 0 }, { x: 10, y: 10 }, { x: 0, y: 10 }, { x: 5, y: 5 },
  ]);
  assert.equal(hull.length, 4);
  assert.ok(polygonArea(hull) > 0, 'counter-clockwise winding gives positive area');
});

test('box mass properties match the analytic rectangle', () => {
  const b = box(0, 0, 10, 5, { density: 2 });
  assert.ok(Math.abs(b.mass - 2 * 20 * 10) < 1e-9);
  const expected = b.mass * (20 ** 2 + 10 ** 2) / 12;
  assert.ok(Math.abs(b.inertia - expected) / expected < 1e-9);
});

test('circle mass properties match the analytic disc', () => {
  const c = circle(0, 0, 4, { density: 3 });
  assert.ok(Math.abs(c.mass - 3 * Math.PI * 16) < 1e-9);
  assert.ok(Math.abs(c.inertia - 0.5 * c.mass * 16) < 1e-9);
});

test('polygon vertices are recentred on the centroid', () => {
  const shape = polygonShape([
    { x: 100, y: 100 }, { x: 140, y: 100 }, { x: 140, y: 120 }, { x: 100, y: 120 },
  ]);
  const c = polygonCentroid(shape.vertices);
  assert.ok(Math.hypot(c.x, c.y) < 1e-9, 'centroid of the recentred shape is the origin');
  assert.ok(Math.hypot(shape.offset.x - 120, shape.offset.y - 110) < 1e-9);
});

test('box local/world transforms round-trip through an angle', () => {
  const b = boxShape(3, 2);
  assert.equal(b.vertices.length, 4);
  assert.equal(b.normals.length, 4);
  for (let i = 0; i < 4; i++) assert.ok(Math.abs(m.len(b.normals[i]) - 1) < 1e-12);
});

test('containsPoint respects rotation', () => {
  const b = box(0, 0, 10, 2, { angle: Math.PI / 2 });
  assert.ok(b.containsPoint(m.v(0, 9)), 'rotated box is now tall');
  assert.ok(!b.containsPoint(m.v(9, 0)));
});
