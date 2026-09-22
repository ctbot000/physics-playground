// Preset scenes. Each one is rebuilt from the current world size, so a resize
// produces a scene that fits rather than a stretched copy of the old one.

import * as m from './math.js';
import { Body, box, boxShape, circle, ngon, polygonShape } from './body.js';
import { DistanceJoint, RevoluteJoint } from './joints.js';
import { pickColor } from './render.js';

/** Attraction constant for the Orbits scene. */
const ORBIT_G = 6;

/** Thickness of the world edges, and how far they poke into view. */
const BOUND_T = 40;
const EDGE = 14;

/**
 * The y of the floor's upper surface. Scenes that guess at this leave a gap
 * under whatever they stand on the floor, and anything small enough rolls out
 * through it -- which reads as a collision bug rather than a layout one.
 */
export function floorTop(height) {
  return height - EDGE;
}

/** Floor and side walls sized to the current world. Every scene starts here. */
export function addBounds(world, { width, height, walls = true, ceiling = false } = {}) {
  const created = [];
  const make = (tag, x, y, hw, hh, friction) =>
    created.push(world.add(box(x, y, hw, hh, {
      isStatic: true, friction, restitution: 0.05, tag, color: '#232a38',
    })));

  make('bounds:floor', width / 2, height + BOUND_T / 2 - EDGE, width / 2 + BOUND_T, BOUND_T / 2, 0.7);
  if (walls) {
    make('bounds:left', -BOUND_T / 2 + EDGE, height / 2, BOUND_T / 2, height, 0.4);
    make('bounds:right', width + BOUND_T / 2 - EDGE, height / 2, BOUND_T / 2, height, 0.4);
  }
  if (ceiling) {
    make('bounds:ceiling', width / 2, -BOUND_T / 2 + EDGE, width / 2 + BOUND_T, BOUND_T / 2, 0.4);
  }
  return created;
}

/**
 * Move the bounds to a new world size in place. A resize regenerates the scene
 * while it is still untouched; once the user has put things in it, the scene is
 * no longer disposable and only the edges move.
 */
export function resizeBounds(world, width, height) {
  for (const body of world.bodies) {
    if (typeof body.tag !== 'string' || !body.tag.startsWith('bounds:')) continue;
    switch (body.tag) {
      case 'bounds:floor':
        body.shape = boxShape(width / 2 + BOUND_T, BOUND_T / 2);
        body.position = m.v(width / 2, height + BOUND_T / 2 - EDGE);
        break;
      case 'bounds:ceiling':
        body.shape = boxShape(width / 2 + BOUND_T, BOUND_T / 2);
        body.position = m.v(width / 2, -BOUND_T / 2 + EDGE);
        break;
      case 'bounds:left':
        body.shape = boxShape(BOUND_T / 2, height);
        body.position = m.v(-BOUND_T / 2 + EDGE, height / 2);
        break;
      case 'bounds:right':
        body.shape = boxShape(BOUND_T / 2, height);
        body.position = m.v(width + BOUND_T / 2 - EDGE, height / 2);
        break;
      default:
        continue;
    }
    body.updateMass();
    body.updateAABB();
  }
}

function anchor(world, x, y, r = 7) {
  return world.add(circle(x, y, r, { isStatic: true, color: '#39445a' }));
}

export const scenes = [
  {
    id: 'sandbox',
    name: 'Sandbox',
    hint: 'Empty room. Pick a tool and start dropping things in.',
    build(world, { width, height }) {
      addBounds(world, { width, height });
    },
  },

  {
    id: 'pyramid',
    name: 'Pyramid',
    hint: 'A friction-held stack. Drag a block out of the bottom row.',
    build(world, { width, height }) {
      addBounds(world, { width, height });
      const size = m.clamp(Math.min(width, height) / 26, 12, 24);
      const rows = m.clamp(Math.floor((height * 0.55) / (size * 2)), 4, 12);
      const baseY = floorTop(height) - size;
      for (let row = 0; row < rows; row++) {
        const count = rows - row;
        const rowY = baseY - row * (size * 2 + 0.6);
        const startX = width / 2 - (count - 1) * (size + 1);
        for (let i = 0; i < count; i++) {
          world.add(box(startX + i * (size * 2 + 2), rowY, size, size, {
            friction: 0.7, restitution: 0.02, color: pickColor(row + i),
          }));
        }
      }
    },
  },

  {
    id: 'tower',
    name: 'Column tower',
    hint: 'Columns and beams, alternating. Pull a column out from the bottom.',
    build(world, { width, height }) {
      addBounds(world, { width, height });
      const unit = m.clamp(Math.min(width, height) / 46, 6, 14);
      const columnHeight = unit * 3;
      const gap = unit * 3;
      const storey = columnHeight * 2 + unit * 2;
      const storeys = m.clamp(Math.floor((height * 0.66) / storey), 3, 9);
      const cx = width / 2;
      const halfBeam = gap + unit * 1.6;

      for (let s = 0; s < storeys; s++) {
        const base = floorTop(height) - s * storey;
        for (const offset of [-gap, 0, gap]) {
          world.add(box(cx + offset, base - columnHeight, unit, columnHeight, {
            friction: 0.85, restitution: 0.01, color: '#5aa9e6',
          }));
        }
        world.add(box(cx, base - columnHeight * 2 - unit, halfBeam, unit, {
          friction: 0.85, restitution: 0.01, color: '#f9a03f',
        }));
      }
    },
    tune: { velocityIterations: 16 },
  },

  {
    id: 'cart',
    name: 'Cart',
    hint: 'Two wheels pinned to a chassis, rolling down bumpy ground.',
    build(world, { width, height, rng }) {
      addBounds(world, { width, height });

      // Descending ground made of overlapping segments, so there is no seam
      // for a wheel to catch on.
      const segments = m.clamp(Math.round(width / 90), 6, 18);
      const segW = width / segments;
      const topY = height * 0.32;
      const dropY = floorTop(height) - 30;
      let previousEnd = topY;
      for (let i = 0; i < segments; i++) {
        const t = (i + 1) / segments;
        const target = topY + (dropY - topY) * t + (rng() - 0.5) * 26;
        const cx = (i + 0.5) * segW;
        const cy = (previousEnd + target) / 2;
        const angle = Math.atan2(target - previousEnd, segW);
        world.add(box(cx, cy + 16, segW * 0.62, 14, {
          isStatic: true, angle, friction: 0.9, restitution: 0.02, color: '#2e3748',
        }));
        previousEnd = target;
      }

      const wheelR = m.clamp(Math.min(width, height) / 26, 14, 30);
      const chassisW = wheelR * 3;
      const cx = width * 0.1;
      const cy = topY - wheelR * 2;
      const chassis = world.add(box(cx, cy, chassisW, wheelR * 0.55, {
        density: 1.1, friction: 0.5, restitution: 0.05, color: '#f9a03f',
      }));
      for (const side of [-1, 1]) {
        const wheel = world.add(circle(cx + side * chassisW * 0.72, cy + wheelR * 0.9, wheelR, {
          density: 1.6, friction: 1, restitution: 0.05, color: '#5aa9e6',
        }));
        wheel.angularDamping = 0.1;
        world.addJoint(new RevoluteJoint(chassis, wheel, m.clone(wheel.position)));
      }

      for (let i = 0; i < 5; i++) {
        world.add(box(width * 0.55 + i * 40, height * 0.1 - i * 26, 14, 14, {
          density: 0.8, friction: 0.6, color: pickColor(i + 4),
        }));
      }
    },
    tune: { velocityIterations: 16 },
  },

  {
    id: 'pendulum',
    name: 'Double pendulum',
    hint: 'Deterministic and unpredictable. Turn trails on.',
    build(world, { width, height }) {
      addBounds(world, { width, height, walls: false });
      const cx = width / 2;
      const cy = height * 0.28;
      const armLength = m.clamp(Math.min(width, height) * 0.22, 60, 190);
      const pivot = anchor(world, cx, cy, 6);

      const arm1 = world.add(box(cx + armLength / 2, cy, armLength / 2, 7, {
        density: 0.9, friction: 0.2, color: '#5aa9e6',
      }));
      const arm2 = world.add(box(cx + armLength * 1.5, cy, armLength / 2, 6, {
        density: 0.9, friction: 0.2, color: '#e07bc8',
      }));
      arm1.linearDamping = 0;
      arm1.angularDamping = 0;
      arm2.linearDamping = 0;
      arm2.angularDamping = 0;

      world.addJoint(new RevoluteJoint(pivot, arm1, m.v(cx, cy)));
      world.addJoint(new RevoluteJoint(arm1, arm2, m.v(cx + armLength, cy)));
    },
    tune: { velocityIterations: 20, linearDamping: 0, angularDamping: 0 },
  },

  {
    id: 'ramps',
    name: 'Ramps',
    hint: 'Rolling versus sliding. Circles win.',
    build(world, { width, height, rng }) {
      addBounds(world, { width, height });
      const levels = m.clamp(Math.floor(height / 120), 2, 5);
      for (let i = 0; i < levels; i++) {
        const y = height * (0.24 + (i * 0.62) / levels);
        const left = i % 2 === 0;
        const tilt = left ? 0.19 : -0.19;
        const cx = left ? width * 0.38 : width * 0.62;
        world.add(box(cx, y, width * 0.34, 8, {
          isStatic: true, angle: tilt, friction: 0.5, color: '#2e3748',
        }));
      }
      for (let i = 0; i < 8; i++) {
        const x = width * 0.14 + rng() * width * 0.12;
        const y = height * 0.1 - i * 34;
        if (i % 2 === 0) world.add(circle(x, y, 13 + rng() * 6, { friction: 0.5, restitution: 0.25 }));
        else world.add(box(x, y, 12 + rng() * 5, 12 + rng() * 5, { friction: 0.5, restitution: 0.1 }));
      }
    },
  },

  {
    id: 'galton',
    name: 'Galton board',
    hint: 'Each peg is a coin flip. The bins fill into a bell curve.',
    build(world, { width, height, rng }) {
      addBounds(world, { width, height });

      // Lay the board out from the space actually available, so the bins never
      // end up below the floor on a short viewport.
      const floorY = floorTop(height);
      const binHeight = m.clamp(height * 0.24, 50, 180);
      const fieldTop = height * 0.2;
      const fieldBottom = floorY - binHeight;
      const fieldHeight = Math.max(80, fieldBottom - fieldTop);
      const rows = m.clamp(Math.floor(fieldHeight / 42), 4, 11);
      const rowGap = fieldHeight / rows;
      const spacing = Math.min(rowGap / 0.85, (width * 0.78) / (rows + 1));
      const pegR = m.clamp(spacing * 0.2, 3, 12);
      const ballR = m.clamp((spacing - 2 * pegR) * 0.36, 2.5, 11);

      for (let row = 0; row < rows; row++) {
        const count = row + 1;
        const y = fieldTop + row * rowGap;
        const startX = width / 2 - ((count - 1) * spacing) / 2;
        for (let i = 0; i < count; i++) {
          world.add(circle(startX + i * spacing, y, pegR, {
            isStatic: true, friction: 0.1, restitution: 0.05, color: '#3c4762',
          }));
        }
      }

      // Side walls run all the way to the top: a ball deflected off the apex
      // has to come back into the field rather than skid off to the wall.
      const spread = (rows * spacing) / 2 + spacing * 0.6;
      for (const side of [-1, 1]) {
        world.add(box(width / 2 + side * spread, fieldBottom / 2, 5, fieldBottom / 2, {
          isStatic: true, friction: 0.1, restitution: 0.05, color: '#2e3748',
        }));
      }

      const binCount = rows + 1;
      const binWidth = (spread * 2 - 10) / binCount;
      for (let i = 0; i <= binCount; i++) {
        world.add(box(width / 2 - spread + 5 + i * binWidth, floorY - binHeight / 2, 1.5, binHeight / 2, {
          isStatic: true, friction: 0.2, color: '#2e3748',
        }));
      }

      const capacity = Math.max(1, Math.floor(binWidth / (2.2 * ballR)) * Math.floor(binHeight / (2.2 * ballR)));
      world.galton = {
        rng,
        x: width / 2,
        y: Math.max(12, fieldTop - rowGap * 1.2),
        jitter: spacing * 0.15,
        radius: ballR,
        total: m.clamp(Math.round(binCount * capacity * 0.62), 40, 150),
        spawned: 0,
        timer: 0,
        interval: 0.06,
      };
    },
    /**
     * Drip the balls in one at a time. Stacking them in a column above the
     * apex would need more height than the canvas has, and the ones at the
     * bottom of it would arrive far too fast to be deflected.
     */
    update(world, dt) {
      const g = world.galton;
      if (!g || g.spawned >= g.total) return;
      g.timer += dt;
      while (g.timer >= g.interval && g.spawned < g.total) {
        g.timer -= g.interval;
        g.spawned++;
        const ball = world.add(circle(g.x + (g.rng() - 0.5) * g.jitter, g.y, g.radius, {
          friction: 0.3, restitution: 0.05, density: 1.4, color: '#f7c948',
        }));
        // Viscous drag. Without it the drop between rows is enough to send a
        // deflected ball clear across several columns, and the board piles up
        // against both outer walls instead of forming a curve.
        ball.linearDamping = 7;
        ball.angularDamping = 7;
      }
    },
    tune: { velocityIterations: 8 },
  },

  {
    id: 'bridge',
    name: 'Rope bridge',
    hint: 'Hinged planks under load. Drop something heavy on it.',
    build(world, { width, height }) {
      addBounds(world, { width, height });
      const y = height * 0.45;
      const segments = m.clamp(Math.floor(width / 46), 8, 26);
      const span = width * 0.82;
      const segW = span / segments / 2;
      const startX = width * 0.09;

      let previous = anchor(world, startX, y, 6);
      const first = previous;
      for (let i = 0; i < segments; i++) {
        const x = startX + segW + i * segW * 2;
        const plank = world.add(box(x, y, segW, 5, {
          density: 0.7, friction: 0.8, restitution: 0.02, color: '#7bc96f',
        }));
        world.addJoint(new RevoluteJoint(previous, plank, m.v(x - segW, y)));
        previous = plank;
      }
      const last = anchor(world, startX + segments * segW * 2, y, 6);
      world.addJoint(new RevoluteJoint(previous, last, m.v(startX + segments * segW * 2, y)));
      void first;

      for (let i = 0; i < 4; i++) {
        world.add(box(width * 0.4 + i * 60, height * 0.12 - i * 30, 22, 22, {
          density: 3.4, friction: 0.6, color: '#f97362',
        }));
      }
    },
    tune: { velocityIterations: 18 },
  },

  {
    id: 'dominoes',
    name: 'Dominoes',
    hint: 'Nudge the leftmost one. Or use the blast tool.',
    build(world, { width, height }) {
      addBounds(world, { width, height });
      const h = m.clamp(height / 12, 26, 54);
      const w = h / 5;
      const gap = h * 0.62;
      const count = Math.floor((width * 0.86) / gap);
      const y = floorTop(height) - h;
      for (let i = 0; i < count; i++) {
        world.add(box(width * 0.08 + i * gap, y, w, h, {
          friction: 0.55, restitution: 0.02, density: 1.2, color: pickColor(i),
        }));
      }
      world.add(circle(width * 0.08 - 90, height * 0.3, 20, { density: 3, friction: 0.5 }));
    },
  },

  {
    id: 'wrecking',
    name: 'Wrecking ball',
    hint: 'Swing the ball into the wall. Drag it back first.',
    build(world, { width, height }) {
      addBounds(world, { width, height });
      const pivot = m.v(width * 0.22, height * 0.14);
      const a = anchor(world, pivot.x, pivot.y, 8);
      const ballR = m.clamp(Math.min(width, height) / 16, 22, 42);
      const ball = world.add(circle(width * 0.22, height * 0.58, ballR, {
        density: 9, friction: 0.5, restitution: 0.15, color: '#8b7bf7',
      }));
      ball.linearDamping = 0;
      world.addJoint(new DistanceJoint(a, ball, pivot, ball.position, { stiffness: 1, damping: 0.05 }));

      const bw = m.clamp(width / 40, 12, 22);
      const cols = 4;
      const rows = m.clamp(Math.floor((height * 0.5) / (bw * 2)), 4, 14);
      const baseX = width * 0.66;
      for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols; c++) {
          world.add(box(baseX + c * (bw * 2 + 1), floorTop(height) - bw - r * (bw * 2 + 0.5), bw, bw, {
            friction: 0.7, restitution: 0.02, color: pickColor(r * 2 + c),
          }));
        }
      }
    },
    tune: { velocityIterations: 16 },
  },

  {
    id: 'cloth',
    name: 'Cloth',
    hint: 'A sprung sheet hung from two corners. Swing it, or drag the ball through it.',
    build(world, { width, height }) {
      addBounds(world, { width, height });
      const cols = m.clamp(Math.floor(width / 46), 8, 18);
      const rows = m.clamp(Math.floor(height / 52), 6, 14);
      const spacing = Math.min((width * 0.6) / cols, (height * 0.58) / rows);
      const startX = width / 2 - ((cols - 1) * spacing) / 2;
      const startY = height * 0.13;
      const r = Math.max(2.5, spacing * 0.15);
      const grid = [];

      for (let row = 0; row < rows; row++) {
        grid[row] = [];
        for (let col = 0; col < cols; col++) {
          // Pinned at the two top corners only, so the sheet sags between them
          // and can be swung, instead of hanging as a flat board.
          const pinned = row === 0 && (col === 0 || col === cols - 1);
          const node = world.add(circle(startX + col * spacing, startY + row * spacing, r, {
            isStatic: pinned, density: 0.9, friction: 0.5, restitution: 0.02,
            color: pinned ? '#39445a' : '#4ec9b0',
          }));
          node.angularDamping = 6;
          grid[row][col] = node;
        }
      }

      // Break force scaled to the load the sheet actually carries. Hung from
      // two corners each top link takes half the sheet, so the threshold has to
      // clear that with room for the transient as it first falls slack.
      const nodeWeight = grid[1][0].mass * m.len(world.gravity);
      const maxForce = nodeWeight * 420;
      const link = (p, q) => {
        world.addJoint(new DistanceJoint(p, q, p.position, q.position, {
          stiffness: 1, damping: 0.4, maxForce,
        }));
      };
      for (let row = 0; row < rows; row++) {
        for (let col = 0; col < cols; col++) {
          if (col + 1 < cols) link(grid[row][col], grid[row][col + 1]);
          if (row + 1 < rows) link(grid[row][col], grid[row + 1][col]);
        }
      }

      // Something to swing the sheet into.
      world.add(circle(width * 0.5, startY + spacing * (rows + 2.5), spacing * 0.9, {
        density: 1.6, friction: 0.6, color: '#f9a03f',
      }));
    },
    tune: { velocityIterations: 16 },
  },

  {
    id: 'orbits',
    name: 'Orbits',
    hint: 'Gravity between the bodies instead of downward. Trails are on.',
    gravity: m.v(0, 0),
    build(world, { width, height, rng }) {
      const cx = width / 2;
      const cy = height / 2;
      const sun = world.add(circle(cx, cy, m.clamp(Math.min(width, height) / 18, 22, 46), {
        density: 26, friction: 0.4, restitution: 0.3, color: '#f7c948', tag: 'sun',
      }));
      sun.linearDamping = 0;

      const count = 9;
      for (let i = 0; i < count; i++) {
        const radius = Math.min(width, height) * (0.14 + (i / count) * 0.33);
        const a = rng() * Math.PI * 2;
        const p = m.v(cx + Math.cos(a) * radius, cy + Math.sin(a) * radius);
        const body = world.add(circle(p.x, p.y, 5 + rng() * 8, {
          density: 1, friction: 0.3, restitution: 0.4, color: pickColor(i * 2),
        }));
        body.linearDamping = 0;
        body.angularDamping = 0;
        // Circular orbit speed for the attraction constant used below.
        const speed = Math.sqrt((ORBIT_G * sun.mass) / radius);
        body.velocity = m.scale(m.perp(m.normalize(m.sub(p, m.v(cx, cy)))), speed);
      }
    },
    update(world) {
      // Mutual attraction, softened so a close pass does not produce an
      // impulse the fixed step cannot represent.
      const bodies = world.bodies.filter((b) => !b.isStatic);
      for (let i = 0; i < bodies.length; i++) {
        for (let j = i + 1; j < bodies.length; j++) {
          const a = bodies[i];
          const b = bodies[j];
          const d = m.sub(b.position, a.position);
          const distSq = m.lenSq(d) + 400;
          const force = (ORBIT_G * a.mass * b.mass) / distSq;
          const dir = m.scale(d, force / Math.sqrt(distSq));
          a.applyForce(dir);
          b.applyForce(m.neg(dir));
        }
      }
    },
    tune: { linearDamping: 0, angularDamping: 0, clampToBounds: false },
    view: { trails: true, grid: false },
  },

  {
    id: 'funnel',
    name: 'Funnel',
    hint: 'A few hundred grains through a narrow throat.',
    build(world, { width, height, rng }) {
      addBounds(world, { width, height });
      const throat = m.clamp(width * 0.045, 18, 42);
      const y = height * 0.56;
      const len = width * 0.42;
      world.add(box(width / 2 - throat - len / 2 * 0.86, y, len / 2, 7, {
        isStatic: true, angle: 0.5, friction: 0.25, color: '#2e3748',
      }));
      world.add(box(width / 2 + throat + len / 2 * 0.86, y, len / 2, 7, {
        isStatic: true, angle: -0.5, friction: 0.25, color: '#2e3748',
      }));

      const count = 170;
      for (let i = 0; i < count; i++) {
        const x = width / 2 + (rng() - 0.5) * width * 0.44;
        const yy = height * 0.3 - Math.floor(i / 14) * 15;
        world.add(circle(x, yy, 5 + rng() * 3, {
          friction: 0.3, restitution: 0.05, density: 1.6, color: pickColor(i),
        }));
      }
    },
    tune: { velocityIterations: 7 },
  },

  {
    id: 'seesaw',
    name: 'Seesaw',
    hint: 'A hinged plank and a pile of weights. Balance it.',
    build(world, { width, height }) {
      addBounds(world, { width, height });
      const cx = width / 2;
      const pivotY = height * 0.72;
      world.add(ngon(cx, pivotY + 26, 3, 30, { isStatic: true, phase: -Math.PI / 2, color: '#2e3748' }));
      const pivot = anchor(world, cx, pivotY, 6);
      const plank = world.add(box(cx, pivotY, width * 0.3, 7, {
        density: 1.4, friction: 0.8, restitution: 0.02, color: '#7bc96f',
      }));
      world.addJoint(new RevoluteJoint(pivot, plank, m.v(cx, pivotY)));

      for (let i = 0; i < 6; i++) {
        world.add(box(cx - width * 0.26 + i * 16, height * 0.2 - i * 40, 15, 15, {
          density: 2, friction: 0.7, color: pickColor(i + 3),
        }));
      }
    },
    tune: { velocityIterations: 16 },
  },
];

export function sceneById(id) {
  return scenes.find((s) => s.id === id) || scenes[0];
}

/** Build a ragdoll-ish chain, used by the chain tool. */
export function buildChain(world, from, to, links = 10, options = {}) {
  const dir = m.sub(to, from);
  const total = m.len(dir);
  if (total < 20) return [];
  const step = total / links;
  const unit = m.scale(dir, 1 / total);
  const created = [];

  let previous = world.add(circle(from.x, from.y, 5, { isStatic: true, color: '#39445a' }));
  created.push(previous);
  for (let i = 1; i <= links; i++) {
    const p = m.add(from, m.scale(unit, step * i));
    const node = world.add(circle(p.x, p.y, Math.max(4, step * 0.22), {
      density: 1, friction: 0.5, restitution: 0.05, color: options.color || '#5aa9e6',
    }));
    world.addJoint(new DistanceJoint(previous, node, previous.position, node.position, {
      stiffness: 1, damping: 0.25,
    }));
    created.push(node);
    previous = node;
  }
  return created;
}

/** Irregular convex blob, used by the polygon tool. */
export function blobBody(x, y, radius, rng, options = {}) {
  const sides = 5 + Math.floor(rng() * 4);
  const points = [];
  for (let i = 0; i < sides; i++) {
    const a = (i / sides) * Math.PI * 2;
    const r = radius * (0.7 + rng() * 0.45);
    points.push({ x: Math.cos(a) * r, y: Math.sin(a) * r });
  }
  return new Body({ ...options, shape: polygonShape(points), position: m.v(x, y) });
}
