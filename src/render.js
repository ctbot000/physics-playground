// Canvas renderer. Owns nothing about the simulation; it only reads it.

import * as m from './math.js';
import { CIRCLE } from './body.js';

export const PALETTE = [
  '#f97362', '#f9a03f', '#f7c948', '#7bc96f',
  '#4ec9b0', '#5aa9e6', '#8b7bf7', '#e07bc8',
];

export function pickColor(seed) {
  return PALETTE[Math.abs(Math.round(seed)) % PALETTE.length];
}

const THEME = {
  background: '#0e1117',
  grid: 'rgba(255, 255, 255, 0.035)',
  gridMajor: 'rgba(255, 255, 255, 0.07)',
  staticFill: '#242b38',
  staticStroke: '#39445a',
  outline: 'rgba(8, 10, 14, 0.85)',
  rope: '#8b96ad',
  rod: '#b9c3d6',
  spring: '#5aa9e6',
  text: '#e6ebf5',
  contact: '#ffd166',
  aabb: 'rgba(90, 169, 230, 0.45)',
  velocity: '#4ec9b0',
  spark: '#ffd88a',
};

export class Renderer {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d', { alpha: false });
    this.dpr = 1;
    this.width = 0;
    this.height = 0;
    this.options = {
      grid: true,
      contacts: false,
      aabb: false,
      velocities: false,
      trails: false,
      outlines: true,
    };
    this.trails = new Map();
  }

  /**
   * Size the backing store to the element's CSS box. Idempotent: assigning
   * `canvas.width` clears the bitmap and resets every context property even
   * when the value is unchanged, so a handler that did it unconditionally
   * would erase the frame it was called to preserve.
   *
   * Returns true when the bitmap was actually resized, so the caller knows it
   * must repaint.
   */
  resize(cssWidth, cssHeight, dpr = window.devicePixelRatio || 1) {
    // A hidden or collapsed surface measures zero, and everything derived from
    // that measurement inherits the collapse. Floor it to something usable.
    const w = Math.max(240, Math.round(cssWidth));
    const h = Math.max(180, Math.round(cssHeight));
    const scale = m.clamp(dpr, 1, 2.5);
    const pixelW = Math.round(w * scale);
    const pixelH = Math.round(h * scale);

    this.width = w;
    this.height = h;
    this.dpr = scale;

    if (this.canvas.width === pixelW && this.canvas.height === pixelH) return false;
    this.canvas.width = pixelW;
    this.canvas.height = pixelH;
    this.canvas.style.width = `${w}px`;
    this.canvas.style.height = `${h}px`;
    return true;
  }

  clear() {
    const { ctx } = this;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.fillStyle = THEME.background;
    ctx.fillRect(0, 0, this.width, this.height);
  }

  drawGrid(spacing = 40) {
    if (!this.options.grid) return;
    const { ctx } = this;
    ctx.lineWidth = 1;
    for (let x = 0; x <= this.width; x += spacing) {
      ctx.strokeStyle = x % (spacing * 5) === 0 ? THEME.gridMajor : THEME.grid;
      ctx.beginPath();
      ctx.moveTo(x + 0.5, 0);
      ctx.lineTo(x + 0.5, this.height);
      ctx.stroke();
    }
    for (let y = 0; y <= this.height; y += spacing) {
      ctx.strokeStyle = y % (spacing * 5) === 0 ? THEME.gridMajor : THEME.grid;
      ctx.beginPath();
      ctx.moveTo(0, y + 0.5);
      ctx.lineTo(this.width, y + 0.5);
      ctx.stroke();
    }
  }

  draw(world, extras = {}) {
    this.clear();
    this.drawGrid();
    if (this.options.trails) this.drawTrails(world);
    this.drawJoints(world);
    for (const body of world.bodies) this.drawBody(body);
    this.drawImpacts(world);
    if (this.options.aabb) this.drawAABBs(world);
    if (this.options.contacts) this.drawContacts(world);
    if (this.options.velocities) this.drawVelocities(world);
    if (extras.preview) this.drawPreview(extras.preview);
    if (extras.highlight) this.drawHighlight(extras.highlight);
  }

  bodyPath(body) {
    const { ctx } = this;
    ctx.beginPath();
    if (body.shape.type === CIRCLE) {
      ctx.arc(body.position.x, body.position.y, body.shape.radius, 0, Math.PI * 2);
    } else {
      const verts = body.worldVertices();
      ctx.moveTo(verts[0].x, verts[0].y);
      for (let i = 1; i < verts.length; i++) ctx.lineTo(verts[i].x, verts[i].y);
      ctx.closePath();
    }
  }

  drawBody(body) {
    const { ctx } = this;
    const fill = body.isStatic ? THEME.staticFill : body.color || pickColor(body.id * 3);

    this.bodyPath(body);
    ctx.fillStyle = fill;
    ctx.fill();

    if (this.options.outlines) {
      ctx.lineWidth = body.isStatic ? 1.5 : 1.25;
      ctx.strokeStyle = body.isStatic ? THEME.staticStroke : THEME.outline;
      ctx.stroke();
    }

    // A spoke makes rotation legible; without it a rolling disc looks like a
    // sliding one.
    if (body.shape.type === CIRCLE && !body.isStatic) {
      const tip = body.toWorld(m.v(body.shape.radius * 0.82, 0));
      ctx.beginPath();
      ctx.moveTo(body.position.x, body.position.y);
      ctx.lineTo(tip.x, tip.y);
      ctx.lineWidth = Math.max(1.2, body.shape.radius * 0.1);
      ctx.strokeStyle = 'rgba(8, 10, 14, 0.5)';
      ctx.lineCap = 'round';
      ctx.stroke();
      ctx.lineCap = 'butt';
    }
  }

  drawJoints(world) {
    const { ctx } = this;
    for (const joint of world.joints) {
      const [p1, p2] = joint.anchors();
      if (joint.kind === 'spring') {
        this.drawSpring(p1, p2);
        continue;
      }
      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      if (joint.kind === 'mouse') {
        ctx.setLineDash([5, 5]);
        ctx.strokeStyle = THEME.contact;
        ctx.lineWidth = 1.5;
      } else if (joint.kind === 'rope') {
        ctx.strokeStyle = THEME.rope;
        ctx.lineWidth = 2;
      } else {
        ctx.strokeStyle = THEME.rod;
        ctx.lineWidth = joint.kind === 'revolute' ? 1 : 2.5;
      }
      ctx.stroke();
      ctx.setLineDash([]);

      if (joint.kind === 'revolute') {
        ctx.beginPath();
        ctx.arc(p1.x, p1.y, 3.5, 0, Math.PI * 2);
        ctx.fillStyle = THEME.rod;
        ctx.fill();
      }
    }
  }

  drawSpring(p1, p2) {
    const { ctx } = this;
    const d = m.sub(p2, p1);
    const length = m.len(d);
    if (length < 1) return;
    const dir = m.scale(d, 1 / length);
    const normal = m.perp(dir);
    const coils = m.clamp(Math.round(length / 14), 4, 24);
    const amplitude = 6;

    ctx.beginPath();
    ctx.moveTo(p1.x, p1.y);
    for (let i = 1; i < coils; i++) {
      const t = i / coils;
      const side = i % 2 === 0 ? 1 : -1;
      const base = m.add(p1, m.scale(dir, length * t));
      const off = m.add(base, m.scale(normal, amplitude * side));
      ctx.lineTo(off.x, off.y);
    }
    ctx.lineTo(p2.x, p2.y);
    ctx.strokeStyle = THEME.spring;
    ctx.lineWidth = 2;
    ctx.stroke();
  }

  drawImpacts(world) {
    const { ctx } = this;
    for (const impact of world.impacts) {
      const remaining = impact.expires - world.time;
      if (remaining <= 0) continue;
      const t = m.clamp(remaining / 0.45, 0, 1);
      const radius = 4 + (1 - t) * Math.min(26, impact.speed / 26);
      ctx.beginPath();
      ctx.arc(impact.point.x, impact.point.y, radius, 0, Math.PI * 2);
      ctx.strokeStyle = THEME.spark;
      ctx.globalAlpha = t * 0.7;
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.globalAlpha = 1;
    }
  }

  drawTrails(world) {
    const { ctx } = this;
    const now = world.time;
    for (const [id, points] of this.trails) {
      if (points.length < 2) continue;
      ctx.beginPath();
      ctx.moveTo(points[0].x, points[0].y);
      for (let i = 1; i < points.length; i++) ctx.lineTo(points[i].x, points[i].y);
      ctx.strokeStyle = pickColor(id * 3);
      ctx.globalAlpha = 0.35;
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.globalAlpha = 1;
    }
    void now;
  }

  /**
   * Trails are sampled and retired on the world clock, so they expire whether
   * or not frames keep arriving.
   */
  sampleTrails(world) {
    const lifetime = 2.5;
    if (!this.options.trails) {
      if (this.trails.size) this.trails.clear();
      return;
    }
    // Retire first and unconditionally: behind a "only while moving" guard the
    // sweep would stop running exactly when the stale trail is on screen.
    for (const [id, points] of this.trails) {
      while (points.length && points[0].t + lifetime < world.time) points.shift();
      if (!points.length) this.trails.delete(id);
    }
    for (const body of world.bodies) {
      if (body.isStatic) continue;
      if (m.lenSq(body.velocity) < 400) continue;
      let points = this.trails.get(body.id);
      if (!points) {
        points = [];
        this.trails.set(body.id, points);
      }
      const last = points[points.length - 1];
      if (!last || m.dist(last, body.position) > 4) {
        points.push({ x: body.position.x, y: body.position.y, t: world.time });
      }
      if (points.length > 90) points.shift();
    }
  }

  drawAABBs(world) {
    const { ctx } = this;
    ctx.strokeStyle = THEME.aabb;
    ctx.lineWidth = 1;
    for (const body of world.bodies) {
      const a = body.aabb;
      ctx.strokeRect(a.minX, a.minY, a.maxX - a.minX, a.maxY - a.minY);
    }
  }

  drawContacts(world) {
    const { ctx } = this;
    for (const arb of world.arbiters.values()) {
      for (const c of arb.contacts) {
        ctx.beginPath();
        ctx.arc(c.point.x, c.point.y, 3, 0, Math.PI * 2);
        ctx.fillStyle = THEME.contact;
        ctx.fill();

        const tip = m.add(c.point, m.scale(arb.normal, 14));
        ctx.beginPath();
        ctx.moveTo(c.point.x, c.point.y);
        ctx.lineTo(tip.x, tip.y);
        ctx.strokeStyle = THEME.contact;
        ctx.lineWidth = 1.25;
        ctx.stroke();
      }
    }
  }

  drawVelocities(world) {
    const { ctx } = this;
    ctx.strokeStyle = THEME.velocity;
    ctx.lineWidth = 1.5;
    for (const body of world.bodies) {
      if (body.isStatic || m.lenSq(body.velocity) < 25) continue;
      const tip = m.add(body.position, m.scale(body.velocity, 0.12));
      ctx.beginPath();
      ctx.moveTo(body.position.x, body.position.y);
      ctx.lineTo(tip.x, tip.y);
      ctx.stroke();
    }
  }

  drawPreview(preview) {
    const { ctx } = this;
    ctx.save();
    ctx.globalAlpha = 0.55;
    ctx.setLineDash([6, 4]);
    ctx.strokeStyle = preview.color || '#8fa3c4';
    ctx.lineWidth = 1.75;
    ctx.beginPath();
    if (preview.type === 'circle') {
      ctx.arc(preview.x, preview.y, Math.max(preview.radius, 1), 0, Math.PI * 2);
    } else if (preview.type === 'rect') {
      ctx.rect(preview.x - preview.halfWidth, preview.y - preview.halfHeight, preview.halfWidth * 2, preview.halfHeight * 2);
    } else if (preview.type === 'line') {
      ctx.moveTo(preview.x1, preview.y1);
      ctx.lineTo(preview.x2, preview.y2);
    } else if (preview.type === 'points' && preview.points.length) {
      ctx.moveTo(preview.points[0].x, preview.points[0].y);
      for (const p of preview.points.slice(1)) ctx.lineTo(p.x, p.y);
      if (preview.closed) ctx.closePath();
    }
    ctx.stroke();
    ctx.restore();
  }

  drawHighlight(body) {
    const { ctx } = this;
    ctx.save();
    this.bodyPath(body);
    ctx.strokeStyle = '#ffffff';
    ctx.globalAlpha = 0.8;
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.restore();
  }
}
