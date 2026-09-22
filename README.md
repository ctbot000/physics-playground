# Physics Playground

A 2D rigid-body physics sandbox that runs in the browser. Drop shapes in, tie
them together, push them around, and watch it all fall over.

**[Open the playground →](https://ctbot000.github.io/physics-playground/)**

No dependencies, no build step, no bundler. The whole thing is ES modules loaded
straight from the page — the engine, the renderer, and the UI are about 2,000
lines of plain JavaScript.

## What's in it

**Fourteen scenes** — a sandbox, a friction-held pyramid, a column tower, a cart
on bumpy ground, a double pendulum, cascading ramps, a Galton board, a hinged
rope bridge, dominoes, a wrecking ball, a sprung cloth sheet, mutual-gravity
orbits, a grain funnel, and a seesaw.

**Nine tools** — drag and throw, drop balls, boxes and irregular blobs, draw
static walls, hang chains, link bodies with springs and pins, blast things
apart, and erase.

**Live controls** — gravity (including upward), bounciness, friction, time scale
and solver iterations, plus overlays for contact points, bounding boxes,
velocity vectors and motion trails.

Keyboard: `Space` pause, `.` single step, `R` reset, `1`–`9` tools,
`Backspace` clear loose bodies, `G` flip gravity.

## The engine

| File | What it does |
| --- | --- |
| [`src/math.js`](src/math.js) | Vectors, 2x2 block inverse, seeded PRNG |
| [`src/body.js`](src/body.js) | Shapes, convex hulls, mass and inertia |
| [`src/collide.js`](src/collide.js) | SAT, manifold clipping, ray casts |
| [`src/joints.js`](src/joints.js) | Revolute, distance/spring/rope, mouse |
| [`src/world.js`](src/world.js) | Broad phase, contact cache, solver |
| [`src/render.js`](src/render.js) | Canvas drawing and debug overlays |
| [`src/scenes.js`](src/scenes.js) | The preset scenes |
| [`src/app.js`](src/app.js) | Canvas sizing, input, controls, loop |

It is an impulse-based sequential solver with persistent contacts, in the
Box2D lineage:

- **Fixed timestep** of 1/120 s with an accumulator, so behaviour does not
  change with the display's refresh rate.
- **Broad phase** is a sort-and-sweep on x; **narrow phase** is SAT with
  Sutherland–Hodgman clipping, producing up to two contact points per pair.
- **Warm starting**: accumulated normal and tangent impulses persist between
  steps, matched by a contact feature id. This is what lets a tall stack
  converge in a handful of iterations.
- **Coulomb friction**: the tangential impulse is capped by the normal impulse
  of the same step, never by scaling the tangential velocity.
- **Penetration** is removed by a separate position pass, not by a velocity
  bias, so bouncing does not accumulate energy.
- **Joints** are solved as implicit soft constraints, with "rigid" expressed as
  a very high frequency rather than as an infinitely stiff one.

### Four things that were not obvious

Each of these produced a visible bug first and a test second.

**Two contacts in one manifold must not share a feature id.** Clipping can hand
both surviving points the same originating edge. They then warm-start from the
same stored impulse, one side of a resting box is loaded harder than the other,
and an eight-box stack leans a degree per second until it topples. Stamping the
contact's position along the reference face into the id fixed it outright.

**A rigid distance joint solved with a plain Baumgarte bias diverges in a
network with loops.** It is fine alone and fine in a chain. In a cloth grid it
reaches a thousand times the mesh's own weight in constraint force — and adding
solver iterations makes it *worse*, which is the tell. Solving "rigid" as a
40 Hz critically-damped soft constraint keeps the length exact to 0.1 px and
settles the same grid completely.

**Restitution measured at the bottom of the overlap returns more energy than
went in.** A discrete step overshoots the surface and the body keeps
accelerating over that extra depth; reflecting that speed and then correcting
the position hands the depth back as height, so a perfectly elastic ball climbs
higher on every bounce. Discounting the gravity picked up while the body was
below the surface puts the error on the safe side.

**A gravity-driven Galton board over-spreads into a bimodal pile.** With a 45 px
drop between rows, a ball deflected off a peg carries enough sideways speed to
cross several columns, and the bins fill at the two outer walls with nothing in
the middle. Viscous drag on the balls — a terminal velocity rather than free
fall — is what makes the quincunx behave like one.

## Running it locally

The page uses ES modules, so it needs to be served over HTTP; opening
`index.html` from the filesystem will not work.

```bash
git clone https://github.com/ctbot000/physics-playground.git
cd physics-playground
npm start
```

Then open <http://localhost:4173>. `npm start` is a 30-line Python static server
that sends no-cache headers, so an edited module is picked up by a plain reload.
Any other static server works too.

## Tests

```bash
npm test
```

58 tests on Node's built-in runner, no test framework. They cover the vector and
mass-property maths, and then assert on behaviour: that a box settles on the
ground without sinking, that a ball rolls down the shallowest slope instead of
stalling, that a box grips a slope below the friction angle, that an eight-box
stack stays stacked and stops moving, that no body ever escapes the world
bounds, that an elastic ball never gains height, that physics matches between
60 Hz and 30 Hz, and that every scene settles at four different viewport sizes.

## Known limits

- **No continuous collision detection.** A small body moving faster than about
  its own radius per substep can tunnel through a thin wall. A hard clamp keeps
  everything inside the world as a backstop.
- **Newton's cradle does not work**, and is not included. A sequential-impulse
  solver cannot reproduce it: the impulse propagates one pair at a time, so
  three balls swing back instead of one swinging out. No amount of iterations or
  substeps fixes it — it needs a different formulation.
- **Sequential impulses are order-dependent.** Very heavy bodies resting on very
  light ones converge slowly; raise the solver iteration slider if a scene
  looks soft.
- **No sleeping.** Every body is solved every step, so a few hundred is the
  practical ceiling before the frame budget bites.

## License

[MIT](LICENSE).
