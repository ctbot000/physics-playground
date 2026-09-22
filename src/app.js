// Wiring: canvas, input, controls, loop. Nothing here knows how the solver
// works; it only drives the world and reflects it into the page.

import * as m from './math.js';
import { World } from './world.js';
import { box, circle } from './body.js';
import { DistanceJoint, RevoluteJoint } from './joints.js';
import { Renderer, pickColor } from './render.js';
import { scenes, sceneById, resizeBounds, buildChain, blobBody } from './scenes.js';

const TOOLS = [
  { id: 'drag', name: 'Drag', glyph: '✥', hint: 'Grab a body and throw it. Right-drag does this with any tool selected.' },
  { id: 'ball', name: 'Ball', glyph: '●', hint: 'Press and drag to set the radius, release to drop it in.' },
  { id: 'box', name: 'Box', glyph: '■', hint: 'Press and drag out a rectangle, release to drop it in.' },
  { id: 'blob', name: 'Blob', glyph: '⬟', hint: 'An irregular convex polygon. Drag to size it.' },
  { id: 'wall', name: 'Wall', glyph: '▬', hint: 'Drag out a static platform. It never moves.' },
  { id: 'chain', name: 'Chain', glyph: '⛓', hint: 'Drag from a fixed point to hang a chain of links.' },
  { id: 'spring', name: 'Spring', glyph: '〜', hint: 'Drag between two bodies to link them. End on empty space to pin to the world.' },
  { id: 'blast', name: 'Blast', glyph: '✳', hint: 'Click to push everything nearby away from the pointer.' },
  { id: 'erase', name: 'Erase', glyph: '✕', hint: 'Click or sweep across bodies to delete them.' },
];

const DEFAULTS = { restitution: 0.2, friction: 0.45, gravity: 900, timeScale: 1, iterations: 10 };
const STORE_KEY = 'physics-playground.v1';

/** localStorage can throw or come back empty; keep an in-memory mirror so a
 *  swallowed write is never contradicted by the next read. */
const prefs = (() => {
  let memory = {};
  try {
    const raw = window.localStorage.getItem(STORE_KEY);
    if (raw) memory = JSON.parse(raw) || {};
  } catch {
    memory = {};
  }
  return {
    get(key, fallback) {
      return key in memory ? memory[key] : fallback;
    },
    set(key, value) {
      memory[key] = value;
      try {
        window.localStorage.setItem(STORE_KEY, JSON.stringify(memory));
      } catch {
        // Private mode or blocked storage: the mirror above still answers reads.
      }
    },
  };
})();

const $ = (id) => document.getElementById(id);

class Playground {
  constructor() {
    this.stage = $('stage');
    this.canvas = $('canvas');
    this.renderer = new Renderer(this.canvas);
    this.world = new World({ gravity: m.v(0, DEFAULTS.gravity) });

    this.scene = sceneById(prefs.get('scene', 'pyramid'));
    this.tool = prefs.get('tool', 'drag');
    this.paused = false;
    this.timeScale = DEFAULTS.timeScale;
    this.material = { restitution: DEFAULTS.restitution, friction: DEFAULTS.friction };
    this.materialOverridden = false;
    // The user's gravity, kept apart from a scene's own. Reading it back off
    // the slider would let a zero-gravity scene redefine "normal" for whatever
    // is loaded next.
    this.userGravity = DEFAULTS.gravity;
    this.touched = false;
    this.rng = m.mulberry32((Date.now() ^ 0x9e3779b9) >>> 0);

    this.pointer = null;
    this.grabJoint = null;
    this.hover = null;
    this.preview = null;

    this.lastFrame = 0;
    this.fps = 0;
    this.stepMs = 0;
    this.singleStep = false;
    this.frameHandle = 0;

    this.buildControls();
    // Build eagerly. Deferring the first layout and draw to the animation loop
    // leaves a hidden or backgrounded tab showing nothing at all, because a
    // hidden surface is never sent a frame.
    this.measure();
    this.loadScene(this.scene.id);
    this.bindInput();
    this.syncUI();
    this.draw();
    this.start();
  }

  // ---------------------------------------------------------------- layout

  measure() {
    const rect = this.stage.getBoundingClientRect();
    // A collapsed or not-yet-laid-out pane measures zero; floor it so the world
    // is small rather than degenerate.
    const changed = this.renderer.resize(rect.width, rect.height);
    this.world.setBounds({ x: 0, y: 0, width: this.renderer.width, height: this.renderer.height });
    return changed;
  }

  handleResize() {
    const changed = this.measure();
    if (!changed) return;
    if (this.touched) {
      resizeBounds(this.world, this.renderer.width, this.renderer.height);
    } else {
      // Nothing in the scene is worth preserving yet, and a scene regenerated
      // for the new shape fits far better than one stretched into it.
      this.loadScene(this.scene.id, { keepView: true });
    }
    // Assigning canvas.width cleared the bitmap; repaint what it destroyed.
    this.draw();
  }

  // ---------------------------------------------------------------- scenes

  loadScene(id, { keepView = false } = {}) {
    const scene = sceneById(id);
    this.scene = scene;
    prefs.set('scene', scene.id);

    const world = this.world;
    world.clear();
    Object.assign(world, {
      gravity: scene.gravity ? m.clone(scene.gravity) : m.v(0, this.userGravity),
      linearDamping: 0.02,
      angularDamping: 0.02,
      velocityIterations: DEFAULTS.iterations,
      positionIterations: 4,
      clampToBounds: true,
    });
    if (scene.tune) Object.assign(world, scene.tune);
    world.onPreStep = scene.update ? (dt) => scene.update(world, dt) : null;

    this.materialOverridden = false;
    this.material = { restitution: DEFAULTS.restitution, friction: DEFAULTS.friction };
    this.touched = false;
    this.grabJoint = null;
    this.rng = m.mulberry32((Date.now() ^ (scene.id.length * 2654435761)) >>> 0);

    scene.build(world, {
      width: this.renderer.width,
      height: this.renderer.height,
      rng: this.rng,
    });

    this.applyViewOptions(scene, { keepView });
    this.syncUI();
  }

  clearLoose() {
    const doomed = this.world.bodies.filter((b) => !b.isStatic);
    for (const body of doomed) this.world.remove(body);
    this.touched = true;
    this.syncUI();
    this.draw();
  }

  // ----------------------------------------------------------------- loop

  start() {
    const frame = (now) => {
      this.frameHandle = requestAnimationFrame(frame);
      this.tick(now);
    };
    this.frameHandle = requestAnimationFrame(frame);
  }

  tick(now) {
    // Frame deltas are wall-clock and clamped: a long stall must not be
    // integrated in one lump, and must not be treated as a frame count either.
    const last = this.lastFrame || now;
    const raw = (now - last) / 1000;
    this.lastFrame = now;
    const delta = Number.isFinite(raw) ? m.clamp(raw, 0, 0.1) : 0;
    if (delta > 0) this.fps = this.fps ? this.fps * 0.9 + (1 / delta) * 0.1 : 1 / delta;

    if (!this.paused || this.singleStep) {
      const dt = this.singleStep ? this.world.fixedDt : delta * this.timeScale;
      const t0 = performance.now();
      const steps = this.singleStep ? this.forceStep() : this.world.update(dt);
      if (steps > 0) this.stepMs = this.stepMs * 0.85 + (performance.now() - t0) * 0.15;
      this.singleStep = false;
    }

    this.renderer.sampleTrails(this.world);
    this.draw();
    this.updateStats();
  }

  /** One physics step regardless of the accumulator, for the Step button. */
  forceStep() {
    this.world.retireImpacts();
    this.world.step(this.world.fixedDt);
    return 1;
  }

  draw() {
    this.renderer.draw(this.world, { preview: this.preview, highlight: this.hover });
  }

  // ------------------------------------------------------------------ UI

  buildControls() {
    const sceneList = $('scene-list');
    sceneList.replaceChildren(...scenes.map((scene) => {
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'chip';
      btn.textContent = scene.name;
      btn.dataset.scene = scene.id;
      btn.addEventListener('click', () => {
        this.loadScene(scene.id);
        this.draw();
        this.closePanelOnNarrow();
      });
      return btn;
    }));

    const toolList = $('tool-list');
    toolList.replaceChildren(...TOOLS.map((tool, index) => {
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'chip';
      btn.dataset.tool = tool.id;
      btn.title = `${tool.name} (${index + 1})`;
      const glyph = document.createElement('span');
      glyph.className = 'glyph';
      glyph.textContent = tool.glyph;
      const label = document.createElement('span');
      label.textContent = tool.name;
      btn.append(glyph, label);
      btn.addEventListener('click', () => this.setTool(tool.id));
      return btn;
    }));

    // Every control writes to the model and then syncs the page from the same
    // handler. Reflecting state only from the frame loop cannot paint a change
    // that stops the frames -- the pause-on-hide badge being the sharp case.
    const slider = (inputId, apply) => {
      const input = $(inputId);
      input.addEventListener('input', () => {
        apply(parseFloat(input.value));
        this.syncUI();
      });
      return input;
    };

    this.inGravity = slider('in-gravity', (value) => {
      this.userGravity = value;
      this.world.gravity = m.v(this.world.gravity.x, value);
    });
    this.inRestitution = slider('in-restitution', (value) => {
      this.material.restitution = value;
      this.materialOverridden = true;
      this.applyMaterial();
    });
    this.inFriction = slider('in-friction', (value) => {
      this.material.friction = value;
      this.materialOverridden = true;
      this.applyMaterial();
    });
    this.inTimeScale = slider('in-timescale', (value) => {
      this.timeScale = value;
    });
    this.inIterations = slider('in-iterations', (value) => {
      this.world.velocityIterations = Math.round(value);
    });

    const toggle = (id, key) => {
      const input = $(id);
      input.checked = prefs.get(key, this.renderer.options[key] ?? input.checked);
      input.addEventListener('change', () => {
        if (key === 'stats') $('stats').hidden = !input.checked;
        else this.renderer.options[key] = input.checked;
        prefs.set(key, input.checked);
        this.syncUI();
        this.draw();
      });
      return input;
    };
    this.tgGrid = toggle('tg-grid', 'grid');
    this.tgTrails = toggle('tg-trails', 'trails');
    this.tgContacts = toggle('tg-contacts', 'contacts');
    this.tgAabb = toggle('tg-aabb', 'aabb');
    this.tgVelocities = toggle('tg-velocities', 'velocities');
    this.tgStats = toggle('tg-stats', 'stats');
    this.renderer.options.grid = this.tgGrid.checked;
    this.renderer.options.trails = this.tgTrails.checked;
    this.renderer.options.contacts = this.tgContacts.checked;
    this.renderer.options.aabb = this.tgAabb.checked;
    this.renderer.options.velocities = this.tgVelocities.checked;
    $('stats').hidden = !this.tgStats.checked;

    $('btn-play').addEventListener('click', () => this.setPaused(!this.paused));
    $('btn-step').addEventListener('click', () => {
      this.setPaused(true);
      this.singleStep = true;
    });
    $('btn-reset').addEventListener('click', () => {
      this.loadScene(this.scene.id);
      this.draw();
    });
    $('btn-panel').addEventListener('click', () => {
      const panel = $('panel');
      const open = panel.classList.toggle('open');
      $('btn-panel').setAttribute('aria-expanded', String(open));
    });
  }

  closePanelOnNarrow() {
    if (window.matchMedia('(max-width: 860px)').matches) {
      $('panel').classList.remove('open');
      $('btn-panel').setAttribute('aria-expanded', 'false');
    }
  }

  setTool(id) {
    this.tool = id;
    prefs.set('tool', id);
    this.syncUI();
  }

  setPaused(paused) {
    this.paused = paused;
    // Reset the frame clock so resuming does not integrate the whole pause.
    this.lastFrame = 0;
    this.syncUI();
    this.draw();
  }

  /**
   * Restore the saved view preferences, then let the scene override them. The
   * override never reaches storage, so leaving a scene that forced trails on
   * gives the user their own setting back rather than keeping the scene's.
   */
  applyViewOptions(scene, { keepView }) {
    const boxes = {
      grid: this.tgGrid, trails: this.tgTrails, contacts: this.tgContacts,
      aabb: this.tgAabb, velocities: this.tgVelocities,
    };
    if (!keepView) {
      for (const [key, input] of Object.entries(boxes)) {
        input.checked = prefs.get(key, input.checked);
      }
    }
    for (const [key, input] of Object.entries(boxes)) {
      this.renderer.options[key] = input.checked;
    }
    this.renderer.trails.clear();

    if (keepView || !scene.view) return;
    for (const [key, value] of Object.entries(scene.view)) {
      this.renderer.options[key] = value;
      // Move the checkbox with it. A renderer option the control disagrees
      // with is a control that lies about what is on screen.
      if (boxes[key]) boxes[key].checked = value;
    }
  }

  applyMaterial() {
    for (const body of this.world.bodies) {
      body.restitution = this.material.restitution;
      body.friction = this.material.friction;
    }
    this.touched = true;
  }

  syncUI() {
    for (const btn of $('scene-list').children) {
      btn.setAttribute('aria-pressed', String(btn.dataset.scene === this.scene.id));
    }
    for (const btn of $('tool-list').children) {
      btn.setAttribute('aria-pressed', String(btn.dataset.tool === this.tool));
    }
    $('scene-hint').textContent = this.scene.hint;
    $('tool-hint').textContent = TOOLS.find((t) => t.id === this.tool)?.hint ?? '';

    if (!this.materialOverridden) {
      this.inRestitution.value = String(this.material.restitution);
      this.inFriction.value = String(this.material.friction);
    }
    this.inGravity.value = String(Math.round(this.world.gravity.y));
    this.inIterations.value = String(this.world.velocityIterations);

    $('out-gravity').textContent = `${Math.round(this.world.gravity.y)}`;
    $('out-restitution').textContent = Number(this.inRestitution.value).toFixed(2);
    $('out-friction').textContent = Number(this.inFriction.value).toFixed(2);
    $('out-timescale').textContent = `${this.timeScale.toFixed(2)}x`;
    $('out-iterations').textContent = String(this.world.velocityIterations);

    const play = $('btn-play');
    play.textContent = this.paused ? 'Resume' : 'Pause';
    play.setAttribute('aria-pressed', String(this.paused));
    $('paused-badge').hidden = !this.paused;
  }

  updateStats() {
    if ($('stats').hidden) return;
    $('stat-fps').textContent = this.fps ? this.fps.toFixed(0) : '--';
    $('stat-bodies').textContent = String(this.world.bodies.length);
    $('stat-contacts').textContent = String(this.world.stats.contacts);
    $('stat-joints').textContent = String(this.world.joints.length);
    $('stat-step').textContent = this.stepMs.toFixed(1);
  }

  // --------------------------------------------------------------- input

  pointFrom(event) {
    const rect = this.canvas.getBoundingClientRect();
    return m.v(event.clientX - rect.left, event.clientY - rect.top);
  }

  bindInput() {
    const canvas = this.canvas;
    canvas.addEventListener('contextmenu', (e) => e.preventDefault());

    canvas.addEventListener('pointerdown', (e) => {
      canvas.setPointerCapture(e.pointerId);
      const point = this.pointFrom(e);
      // The right button always grabs, whatever tool is selected.
      const tool = e.button === 2 ? 'drag' : this.tool;
      this.pointer = { id: e.pointerId, tool, start: point, current: point, moved: false };
      this.beginTool(tool, point);
      this.draw();
    });

    canvas.addEventListener('pointermove', (e) => {
      const point = this.pointFrom(e);
      if (!this.pointer || e.pointerId !== this.pointer.id) {
        this.hover = this.tool === 'erase' || this.tool === 'drag' ? this.world.bodyAt(point) : null;
        return;
      }
      if (m.dist(this.pointer.start, point) > 3) this.pointer.moved = true;
      this.pointer.current = point;
      this.moveTool(this.pointer.tool, point);
    });

    const end = (e) => {
      if (!this.pointer || e.pointerId !== this.pointer.id) return;
      const point = this.pointFrom(e);
      this.endTool(this.pointer.tool, point);
      this.pointer = null;
      this.preview = null;
      this.draw();
    };
    canvas.addEventListener('pointerup', end);
    canvas.addEventListener('pointercancel', end);
    canvas.addEventListener('pointerleave', () => {
      this.hover = null;
    });

    window.addEventListener('keydown', (e) => {
      if (e.target instanceof HTMLInputElement) return;
      if (e.key === ' ') {
        e.preventDefault();
        this.setPaused(!this.paused);
      } else if (e.key === '.') {
        this.setPaused(true);
        this.singleStep = true;
      } else if (e.key === 'r' || e.key === 'R') {
        this.loadScene(this.scene.id);
        this.draw();
      } else if (e.key === 'Backspace') {
        e.preventDefault();
        this.clearLoose();
      } else if (e.key === 'g' || e.key === 'G') {
        this.userGravity = -this.world.gravity.y;
        this.world.gravity = m.v(this.world.gravity.x, this.userGravity);
        this.syncUI();
      } else if (e.key >= '1' && e.key <= '9') {
        const tool = TOOLS[Number(e.key) - 1];
        if (tool) this.setTool(tool.id);
      }
    });

    // The pause has to reach the page from this handler. Becoming hidden is
    // exactly what stops the frames, so a badge painted from the loop could
    // never appear.
    document.addEventListener('visibilitychange', () => {
      if (document.hidden && !this.paused) {
        this.wasAutoPaused = true;
        this.setPaused(true);
      } else if (!document.hidden && this.wasAutoPaused) {
        this.wasAutoPaused = false;
        this.setPaused(false);
      }
    });

    // Fires as soon as the stage has a real box, which is also the moment a
    // previously hidden pane becomes measurable.
    new ResizeObserver(() => this.handleResize()).observe(this.stage);
  }

  spawnOptions(extra = {}) {
    return {
      restitution: this.material.restitution,
      friction: this.material.friction,
      color: pickColor(Math.floor(this.rng() * 8)),
      ...extra,
    };
  }

  beginTool(tool, point) {
    this.touched = true;
    switch (tool) {
      case 'drag': {
        this.grabJoint = this.world.grab(point, { padding: 6 });
        break;
      }
      case 'blast':
        this.world.explode(point, Math.min(this.renderer.width, this.renderer.height) * 0.28, 620);
        break;
      case 'erase': {
        const body = this.world.bodyAt(point);
        if (body) this.world.remove(body);
        break;
      }
      default:
        break;
    }
  }

  moveTool(tool, point) {
    const start = this.pointer.start;
    const reach = m.dist(start, point);
    switch (tool) {
      case 'drag':
        if (this.grabJoint) this.grabJoint.setTarget(point);
        break;
      case 'ball':
        this.preview = { type: 'circle', x: start.x, y: start.y, radius: Math.max(reach, 12) };
        break;
      case 'blob':
        this.preview = { type: 'circle', x: start.x, y: start.y, radius: Math.max(reach, 14) };
        break;
      case 'box':
      case 'wall':
        this.preview = {
          type: 'rect',
          x: (start.x + point.x) / 2,
          y: (start.y + point.y) / 2,
          halfWidth: Math.max(Math.abs(point.x - start.x) / 2, tool === 'wall' ? 8 : 12),
          halfHeight: Math.max(Math.abs(point.y - start.y) / 2, tool === 'wall' ? 5 : 12),
          color: tool === 'wall' ? '#8fa3c4' : undefined,
        };
        break;
      case 'chain':
      case 'spring':
        this.preview = { type: 'line', x1: start.x, y1: start.y, x2: point.x, y2: point.y };
        break;
      case 'erase': {
        const body = this.world.bodyAt(point);
        if (body) this.world.remove(body);
        break;
      }
      default:
        break;
    }
  }

  endTool(tool, point) {
    const start = this.pointer.start;
    const reach = m.dist(start, point);
    const world = this.world;

    switch (tool) {
      case 'drag':
        world.release(this.grabJoint);
        this.grabJoint = null;
        break;

      case 'ball':
        world.add(circle(start.x, start.y, m.clamp(Math.max(reach, 16), 6, 140), this.spawnOptions()));
        break;

      case 'blob':
        world.add(blobBody(start.x, start.y, m.clamp(Math.max(reach, 18), 10, 140), this.rng, this.spawnOptions()));
        break;

      case 'box': {
        const hw = m.clamp(Math.max(Math.abs(point.x - start.x) / 2, 16), 6, 200);
        const hh = m.clamp(Math.max(Math.abs(point.y - start.y) / 2, 16), 6, 200);
        const cx = this.pointer.moved ? (start.x + point.x) / 2 : start.x;
        const cy = this.pointer.moved ? (start.y + point.y) / 2 : start.y;
        world.add(box(cx, cy, hw, hh, this.spawnOptions()));
        break;
      }

      case 'wall': {
        const hw = m.clamp(Math.max(Math.abs(point.x - start.x) / 2, 30), 8, 900);
        const hh = m.clamp(Math.max(Math.abs(point.y - start.y) / 2, 8), 4, 900);
        world.add(box((start.x + point.x) / 2, (start.y + point.y) / 2, hw, hh, {
          isStatic: true, friction: this.material.friction, restitution: this.material.restitution,
        }));
        break;
      }

      case 'chain':
        if (reach > 30) {
          buildChain(world, start, point, m.clamp(Math.round(reach / 34), 3, 24));
        }
        break;

      case 'spring':
        this.linkBodies(start, point);
        break;

      default:
        break;
    }
    this.syncUI();
  }

  /**
   * Spring or pin between what the drag started on and what it ended on. An end
   * on empty space anchors to a new static point instead.
   */
  linkBodies(start, end) {
    const world = this.world;
    const a = world.bodyAt(start, 4);
    if (!a) return;
    let b = world.bodyAt(end, 4);
    if (b === a) return;

    if (!b) {
      b = world.add(circle(end.x, end.y, 6, { isStatic: true, color: '#39445a' }));
    }
    const rigid = m.dist(start, end) < 24;
    if (rigid) {
      world.addJoint(new RevoluteJoint(a, b, m.scale(m.add(start, end), 0.5)));
    } else {
      world.addJoint(new DistanceJoint(a, b, start, end, {
        stiffness: 0.55, damping: 0.25, kind: 'spring',
      }));
    }
  }
}

// Construct as soon as the document has a body to measure, and never from
// inside the frame loop.
function boot() {
  const app = new Playground();
  window.playground = app; // Handy from the console; nothing depends on it.
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', boot, { once: true });
} else {
  boot();
}

export { Playground, TOOLS };
