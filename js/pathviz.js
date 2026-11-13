import * as THREE from 'three';

/* 路径可视化：路径点标记、规划连线、TCP 运行轨迹 */

const MAX_TRAIL = 6000;

function tagSprite(text, color) {
  const pad = 16;
  const c = document.createElement('canvas');
  const ctx = c.getContext('2d');
  const font = '600 40px "PingFang SC", "Microsoft YaHei", system-ui, sans-serif';
  ctx.font = font;
  const w = Math.ceil(ctx.measureText(text).width) + pad * 2;
  c.width = Math.ceil(w / 4) * 4;
  c.height = 72;
  const g = c.getContext('2d');
  g.font = font;
  g.fillStyle = 'rgba(255,255,255,0.94)';
  g.strokeStyle = 'rgba(15,23,42,0.10)';
  g.lineWidth = 2;
  const r = 18;
  const bw = w;
  g.beginPath();
  g.moveTo(r, 6); g.lineTo(bw - r, 6); g.quadraticCurveTo(bw - 4, 6, bw - 4, 6 + r);
  g.lineTo(bw - 4, 66 - r); g.quadraticCurveTo(bw - 4, 66, bw - r, 66);
  g.lineTo(r, 66); g.quadraticCurveTo(4, 66, 4, 66 - r);
  g.lineTo(4, 6 + r); g.quadraticCurveTo(4, 6, r, 6);
  g.closePath(); g.fill(); g.stroke();
  g.fillStyle = color;
  g.beginPath(); g.arc(28, 36, 9, 0, Math.PI * 2); g.fill();
  g.fillStyle = '#0f172a';
  g.textBaseline = 'middle';
  g.fillText(text, 46, 37);

  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false, depthWrite: false }));
  sp.scale.set((c.width / 72) * 0.042, 0.042, 1);
  sp.renderOrder = 20;
  return sp;
}

export class PathViz {
  constructor(scene) {
    this.scene = scene;
    this.root = new THREE.Group();
    this.root.name = 'pathviz';
    scene.add(this.root);

    this.planGroup = new THREE.Group();
    this.markerGroup = new THREE.Group();
    this.root.add(this.planGroup, this.markerGroup);

    this.markers = [];
    this.labels = [];
    this.lines = [];
    this._halos = [];
    this.visible = true;
    this.trailVisible = true;
    this.recording = false;
    this.count = 0;
    this.phaseColor = new THREE.Color(0x0f9d8f);

    // 轨迹
    this.trailPos = new Float32Array(MAX_TRAIL * 3);
    this.trailCol = new Float32Array(MAX_TRAIL * 3);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(this.trailPos, 3).setUsage(THREE.DynamicDrawUsage));
    geo.setAttribute('color', new THREE.BufferAttribute(this.trailCol, 3).setUsage(THREE.DynamicDrawUsage));
    geo.setDrawRange(0, 0);
    this.trailGeo = geo;
    this.trailLine = new THREE.Line(geo, new THREE.LineBasicMaterial({
      vertexColors: true, transparent: true, opacity: 0.95, depthWrite: false,
    }));
    this.trailLine.frustumCulled = false;
    this.trailLine.renderOrder = 5;
    this.root.add(this.trailLine);
    this.lastPoint = null;
  }

  setVisible(v) {
    this.visible = v;
    this.root.visible = v;
  }

  setTrailVisible(v) {
    this.trailVisible = v;
    this.trailLine.visible = v;
  }

  /* ---------------- 规划阶段 ---------------- */
  clearPlan() {
    for (const m of this.markers) {
      this.markerGroup.remove(m);
      if (m.material.map) m.material.map.dispose();
      m.material.dispose();
    }
    for (const l of this.labels) {
      this.markerGroup.remove(l);
      if (l.material.map) l.material.map.dispose();
      l.material.dispose();
    }
    this.markers = [];
    this.labels = [];
    this._halos = [];
    while (this.planGroup.children.length) {
      const l = this.planGroup.children.pop();
      l.geometry.dispose();
      l.material.dispose();
    }
    this.lines = [];
  }

  addWaypoint(pt, index) {
    const color = new THREE.Color(pt.color || 0x0d9488);
    const dot = new THREE.Mesh(
      new THREE.SphereGeometry(0.0095, 16, 12),
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 1 })
    );
    dot.position.copy(pt.pos);
    dot.renderOrder = 8;
    this.markerGroup.add(dot);

    const halo = new THREE.Mesh(
      new THREE.RingGeometry(0.014, 0.018, 32),
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.75, side: THREE.DoubleSide, depthWrite: false })
    );
    halo.position.copy(pt.pos);
    halo.renderOrder = 7;
    this.markerGroup.add(halo);

    const label = tagSprite(`${index + 1} ${pt.label}`, `#${color.getHexString()}`);
    label.position.copy(pt.pos).add(new THREE.Vector3(0, 0, 0.052 + (index % 2) * 0.012));
    this.markerGroup.add(label);
    this.labels.push(label);

    this.markers.push(dot, halo);
    this._halos.push(halo);
    return dot;
  }

  drawPlanLines(points) {
    for (let i = 0; i < points.length - 1; i++) {
      const a = points[i].pos;
      const b = points[i + 1].pos;
      const geo = new THREE.BufferGeometry().setFromPoints([a.clone(), b.clone()]);
      const line = new THREE.Line(geo, new THREE.LineDashedMaterial({
        color: points[i + 1].color || 0x0d9488,
        dashSize: 0.012, gapSize: 0.008, transparent: true, opacity: 0.7, depthWrite: false,
      }));
      line.computeLineDistances();
      line.renderOrder = 6;
      this.planGroup.add(line);
      this.lines.push(line);
    }
  }

  /** 执行中高亮当前段（其余淡化） */
  focusSegment(index, points) {
    this.lines.forEach((l, i) => {
      l.material.opacity = i === index ? 0.95 : 0.25;
    });
    this._halos.forEach((h, i) => { h.material.opacity = i === index ? 0.95 : 0.25; });
    this.labels.forEach((l, i) => { l.material.opacity = i === index ? 1 : 0.4; });
  }

  fadePlan(opacity = 0.35) {
    this.lines.forEach((l) => { l.material.opacity = opacity * 0.7; });
    this._halos.forEach((h) => { h.material.opacity = opacity * 0.55; });
    this.labels.forEach((l) => { l.material.opacity = opacity * 0.85; });
    this.markers.forEach((m) => {
      if (m.geometry.type === 'SphereGeometry') m.material.opacity = 0.4;
    });
  }

  animateHalo(t) {
    if (!this._halos?.length) return;
    const s = 1 + 0.12 * Math.sin(t * 3.2);
    this._halos.forEach((h) => h.scale.setScalar(s));
  }

  /* ---------------- 轨迹 ---------------- */
  beginTrail() {
    this.clearTrail();
    this.recording = true;
  }

  setPhaseColor(hex) {
    this.phaseColor.setHex(hex);
  }

  pushTrail(pos) {
    if (!this.recording || this.count >= MAX_TRAIL) return;
    if (this.lastPoint && this.lastPoint.distanceToSquared(pos) < 4e-6) return;
    const i = this.count++;
    this.trailPos[i * 3] = pos.x;
    this.trailPos[i * 3 + 1] = pos.y;
    this.trailPos[i * 3 + 2] = pos.z;
    this.trailCol[i * 3] = this.phaseColor.r;
    this.trailCol[i * 3 + 1] = this.phaseColor.g;
    this.trailCol[i * 3 + 2] = this.phaseColor.b;
    this.lastPoint = pos.clone();
    this.trailGeo.setDrawRange(0, this.count);
    this.trailGeo.attributes.position.needsUpdate = true;
    this.trailGeo.attributes.color.needsUpdate = true;
    this.trailGeo.computeBoundingSphere();
  }

  endTrail() {
    this.recording = false;
    this.lastPoint = null;
  }

  clearTrail() {
    this.count = 0;
    this.lastPoint = null;
    this.trailGeo.setDrawRange(0, 0);
  }

  trailLength() {
    let len = 0;
    for (let i = 1; i < this.count; i++) {
      const a = new THREE.Vector3().fromArray(this.trailPos, (i - 1) * 3);
      const b = new THREE.Vector3().fromArray(this.trailPos, i * 3);
      len += a.distanceTo(b);
    }
    return len;
  }
}
