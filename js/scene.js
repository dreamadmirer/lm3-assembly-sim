import * as THREE from 'three';
import { BINS, OBJECTS, TABLE } from './config.js';

const V3 = (x, y, z) => new THREE.Vector3(x, y, z);

export function makeLabel(text, color = '#0f172a', bg = 'rgba(255,255,255,0.92)') {
  const c = document.createElement('canvas');
  c.width = 256; c.height = 128;
  const ctx = c.getContext('2d');
  ctx.fillStyle = bg;
  const r = 34;
  ctx.beginPath();
  ctx.moveTo(r, 4); ctx.lineTo(256 - r, 4); ctx.quadraticCurveTo(252, 4, 252, 4 + r);
  ctx.lineTo(252, 124 - r); ctx.quadraticCurveTo(252, 124, 256 - r, 124);
  ctx.lineTo(r, 124); ctx.quadraticCurveTo(4, 124, 4, 124 - r);
  ctx.lineTo(4, 4 + r); ctx.quadraticCurveTo(4, 4, r, 4);
  ctx.closePath(); ctx.fill();
  ctx.fillStyle = color;
  ctx.font = '600 62px "PingFang SC", "Microsoft YaHei", system-ui, sans-serif';
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.fillText(text, 128, 68);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: true, depthWrite: false }));
  sprite.scale.set(0.132, 0.066, 1);
  sprite.renderOrder = 10;
  return sprite;
}

function plastic(color, opts = {}) {
  return new THREE.MeshPhysicalMaterial({
    color, roughness: 0.34, metalness: 0.05, clearcoat: 0.5, clearcoatRoughness: 0.4, ...opts,
  });
}

/* ---------------- 桌面工作台 ---------------- */
function buildTable() {
  const g = new THREE.Group();
  const { width, depth, thickness, center } = TABLE;

  const topMat = new THREE.MeshPhysicalMaterial({ color: 0xeaedf4, roughness: 0.62, metalness: 0.04, clearcoat: 0.18 });
  const top = new THREE.Mesh(new THREE.BoxGeometry(width, depth, thickness), topMat);
  top.position.set(center[0], center[1], -thickness / 2);
  top.castShadow = true; top.receiveShadow = true;
  g.add(top);

  // 台面边缘深色包边
  const edgeMat = new THREE.MeshStandardMaterial({ color: 0x98a3b3, roughness: 0.35, metalness: 0.7 });
  const edge = new THREE.Mesh(new THREE.BoxGeometry(width + 0.008, depth + 0.008, 0.008), edgeMat);
  edge.position.set(center[0], center[1], -thickness + 0.004);
  g.add(edge);

  // 桌腿
  const legMat = new THREE.MeshStandardMaterial({ color: 0x3d444e, roughness: 0.35, metalness: 0.75 });
  const legH = 0.72;
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) {
    const leg = new THREE.Mesh(new THREE.BoxGeometry(0.045, 0.045, legH), legMat);
    leg.position.set(center[0] + sx * (width / 2 - 0.07), center[1] + sy * (depth / 2 - 0.07), -thickness - legH / 2);
    leg.castShadow = true; leg.receiveShadow = true;
    g.add(leg);
  }
  // 横梁
  for (const sy of [-1, 1]) {
    const bar = new THREE.Mesh(new THREE.BoxGeometry(width - 0.1, 0.03, 0.03), legMat);
    bar.position.set(center[0], center[1] + sy * (depth / 2 - 0.07), -thickness - 0.18);
    g.add(bar);
  }
  return g;
}

/* ---------------- 料盒 ---------------- */
function buildBin(cfg) {
  const g = new THREE.Group();
  const [w, d, h] = cfg.size;
  const t = 0.008;
  const mat = new THREE.MeshPhysicalMaterial({ color: cfg.color, roughness: 0.42, metalness: 0.08, clearcoat: 0.35 });
  const inner = new THREE.MeshPhysicalMaterial({ color: cfg.color, roughness: 0.5, metalness: 0.05, side: THREE.BackSide });
  const wallMat = new THREE.MeshPhysicalMaterial({ color: cfg.color, roughness: 0.5, metalness: 0.05 });

  const floor = new THREE.Mesh(new THREE.BoxGeometry(w, d, t), mat);
  floor.position.z = t / 2;
  floor.receiveShadow = true; floor.castShadow = true;
  g.add(floor);

  const mk = (bw, bd, x, y) => {
    const wall = new THREE.Mesh(new THREE.BoxGeometry(bw, bd, h), wallMat);
    wall.position.set(x, y, h / 2);
    wall.castShadow = true; wall.receiveShadow = true;
    g.add(wall);
  };
  mk(w, t, 0, d / 2 - t / 2);
  mk(w, t, 0, -d / 2 + t / 2);
  mk(t, d - 2 * t, w / 2 - t / 2, 0);
  mk(t, d - 2 * t, -w / 2 + t / 2, 0);

  // 内壁提亮（避免内部纯黑）
  const innerBox = new THREE.Mesh(new THREE.BoxGeometry(w - 2 * t, d - 2 * t, h - t), inner);
  innerBox.position.z = (h + t) / 2;
  g.add(innerBox);

  g.position.set(cfg.pos[0], cfg.pos[1], 0);

  const label = makeLabel(`${cfg.name}`, cfg.tint, 'rgba(255,255,255,0.94)');
  label.position.set(0, 0, h + 0.062);
  g.add(label);

  g.userData = { ...cfg, inner: [w - 2 * t, d - 2 * t, t] };
  return g;
}

/* ---------------- 零件 ---------------- */
function buildObject(cfg) {
  const group = new THREE.Group();
  const mat = plastic(cfg.color, { roughness: 0.3, clearcoat: 0.6 });
  let height = 0;
  let mesh;

  switch (cfg.type) {
    case 'cube': {
      const [w, d, h] = cfg.size;
      height = h;
      mesh = new THREE.Mesh(new THREE.BoxGeometry(w, d, h), mat);
      mesh.position.z = h / 2;
      break;
    }
    case 'cylinder': {
      height = cfg.height;
      mesh = new THREE.Mesh(new THREE.CylinderGeometry(cfg.radius, cfg.radius, cfg.height, 36), mat);
      mesh.rotation.x = Math.PI / 2;
      mesh.position.z = cfg.height / 2;
      break;
    }
    case 'hex': {
      height = cfg.height;
      mesh = new THREE.Mesh(new THREE.CylinderGeometry(cfg.radius, cfg.radius, cfg.height, 6), mat);
      mesh.rotation.x = Math.PI / 2;
      mesh.position.z = cfg.height / 2;
      break;
    }
    case 'tri': {
      height = cfg.height;
      mesh = new THREE.Mesh(new THREE.CylinderGeometry(cfg.radius, cfg.radius, cfg.height, 3), mat);
      mesh.rotation.x = Math.PI / 2;
      mesh.position.z = cfg.height / 2;
      break;
    }
    case 'sphere': {
      height = cfg.radius * 2;
      mesh = new THREE.Mesh(new THREE.SphereGeometry(cfg.radius, 32, 24), mat);
      mesh.position.z = cfg.radius;
      break;
    }
    case 'lblock': {
      const [w, d, h] = cfg.size;
      height = h;
      const a = new THREE.Mesh(new THREE.BoxGeometry(w, d * 0.42, h), mat);
      a.position.set(0, -d * 0.29, h / 2);
      const b = new THREE.Mesh(new THREE.BoxGeometry(w * 0.42, d * 0.58, h), mat);
      b.position.set(-w * 0.29, d * 0.21, h / 2);
      const sub = new THREE.Group();
      sub.add(a, b);
      sub.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
      group.add(sub);
      mesh = null;
      break;
    }
    default:
      mesh = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.05, 0.05), mat);
  }

  if (mesh) {
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    group.add(mesh);
  }

  group.position.set(cfg.pos[0], cfg.pos[1], 0);
  group.rotation.z = cfg.yaw || 0;
  group.userData = { cfg, height, inBin: null, busy: false };

  const ring = new THREE.Mesh(
    new THREE.RingGeometry(0.042, 0.05, 48),
    new THREE.MeshBasicMaterial({ color: 0x0f9d8f, transparent: true, opacity: 0, side: THREE.DoubleSide, depthWrite: false })
  );
  ring.position.z = 0.0015;
  group.add(ring);
  group.userData.ring = ring;

  return group;
}

/* ---------------- 场景搭建 ---------------- */
export function buildWorld(scene) {
  scene.background = new THREE.Color(0xe9edf3);
  scene.fog = new THREE.Fog(0xe9edf3, 5, 12);

  const hemi = new THREE.HemisphereLight(0xffffff, 0xd3dae5, 0.8);
  scene.add(hemi);

  const key = new THREE.DirectionalLight(0xfffdf8, 2.9);
  key.position.set(1.7, 1.5, 2.5);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  key.shadow.camera.near = 0.4;
  key.shadow.camera.far = 7;
  key.shadow.camera.left = -1.3;
  key.shadow.camera.right = 1.3;
  key.shadow.camera.top = 1.3;
  key.shadow.camera.bottom = -1.3;
  key.shadow.bias = -0.0004;
  key.shadow.normalBias = 0.022;
  scene.add(key);

  const fill = new THREE.DirectionalLight(0xdfe9ff, 0.55);
  fill.position.set(-2.2, -1.8, 1.4);
  scene.add(fill);

  const rim = new THREE.DirectionalLight(0xffffff, 0.75);
  rim.position.set(-1.0, 2.4, 1.6);
  scene.add(rim);

  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(24, 24),
    new THREE.MeshStandardMaterial({ color: 0xc6cfdc, roughness: 0.95, metalness: 0 })
  );
  ground.position.z = -0.72 - TABLE.thickness;
  ground.receiveShadow = true;
  scene.add(ground);

  const grid = new THREE.GridHelper(12, 60, 0xc8cfda, 0xd8dee7);
  grid.rotation.x = Math.PI / 2;
  grid.position.z = -0.72 - TABLE.thickness + 0.001;
  grid.material.transparent = true;
  grid.material.opacity = 0.55;
  scene.add(grid);

  const table = buildTable();
  scene.add(table);

  const bins = {};
  for (const key of Object.keys(BINS)) {
    bins[key] = buildBin(BINS[key]);
    scene.add(bins[key]);
  }

  const objects = OBJECTS.map((cfg) => {
    const o = buildObject(cfg);
    scene.add(o);
    return o;
  });

  // 目标料盒指示环
  const targetRing = new THREE.Mesh(
    new THREE.RingGeometry(0.168, 0.181, 72),
    new THREE.MeshBasicMaterial({ color: 0x0d9488, transparent: true, opacity: 0.85, side: THREE.DoubleSide, depthWrite: false })
  );
  targetRing.scale.set(1, 0.76, 1);
  targetRing.position.z = 0.0018;
  targetRing.visible = false;
  scene.add(targetRing);

  // 机器人底座下的固定圆盘
  const plate = new THREE.Mesh(
    new THREE.CylinderGeometry(0.085, 0.09, 0.012, 48),
    new THREE.MeshStandardMaterial({ color: 0xb9c1cc, roughness: 0.32, metalness: 0.8 })
  );
  plate.rotation.x = Math.PI / 2;
  plate.position.set(0, -0.28, 0.006);
  plate.receiveShadow = true;
  scene.add(plate);

  return { table, bins, objects, plate, key, targetRing };
}
