import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { Kinematics, JOINTS_MODEL, TOOL_OFFSET, FLANGE_OFFSET } from './kinematics.js';
import { ROBOT_BASE, ROBOT_YAW, POSE_READY } from './config.js';

const easeInOut = (t) => (t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2);
const nextFrame = () => new Promise((r) => requestAnimationFrame(r));

const MAT = {
  // 缎面注塑白（对齐 LEBAI 官方渲染：柔和哑光 + 轻微清漆）
  shell: new THREE.MeshPhysicalMaterial({
    color: 0xfbfcfe, roughness: 0.3, metalness: 0.0,
    clearcoat: 0.35, clearcoatRoughness: 0.25, envMapIntensity: 1.0,
  }),
  base: null, // 与壳体同色，见下方赋值
  // 关节分隔环带：浅银灰（官方图中的灰色关节环）
  band: new THREE.MeshPhysicalMaterial({ color: 0xb6bcc6, roughness: 0.38, metalness: 0.5, clearcoat: 0.25 }),
  flange: new THREE.MeshPhysicalMaterial({ color: 0xcdd3db, roughness: 0.26, metalness: 0.6, clearcoat: 0.25 }),
  dark: new THREE.MeshPhysicalMaterial({ color: 0x2a2f37, roughness: 0.4, metalness: 0.4 }),
  // 夹爪联动臂：乐白夹爪的红色阳极氧化铝
  finger: new THREE.MeshPhysicalMaterial({ color: 0xb5342a, roughness: 0.26, metalness: 0.7, clearcoat: 0.35 }),
  accent: new THREE.MeshPhysicalMaterial({ color: 0xc0392b, roughness: 0.3, metalness: 0.5 }),
  // 基座状态 LED 环（乐白「灯光交互」元素）
  led: new THREE.MeshStandardMaterial({ color: 0x0b3035, emissive: 0x2ad3c8, emissiveIntensity: 1.1, roughness: 0.35, metalness: 0.1 }),
  // 夹爪垫：浅灰阻尼面
  pad: new THREE.MeshStandardMaterial({ color: 0xdadfe6, roughness: 0.55, metalness: 0.05 }),
};
MAT.base = MAT.shell;

export class Robot {
  constructor(scene) {
    this.scene = scene;
    this.kin = new Kinematics(ROBOT_YAW);
    this.q = [0, 0, 0, 0, 0, 0];
    this.root = new THREE.Group();
    this.root.position.set(...ROBOT_BASE);
    scene.add(this.root);

    this.links = [];
    this.gripperOpen = 0.09;
    this.gripperTarget = 0.09;
    this.gripperRate = 0.35;
    this.holder = null;
    this.busy = false;
  }

  async load(url, onProgress) {
    const gltf = await new Promise((resolve, reject) => {
      new GLTFLoader().load(url, resolve, (e) => {
        if (onProgress && e.total) onProgress(Math.min(1, e.loaded / e.total));
      }, reject);
    });
    const byName = new Map();
    gltf.scene.traverse((o) => { if (o.isMesh) byName.set(o.name, o); });

    const order = ['ARM0', 'ARM1', 'ARM2', 'ARM3', 'ARM4', 'ARM5', 'ARM6'];
    for (const name of order) {
      const src = byName.get(name);
      if (!src) continue;
      const mesh = new THREE.Mesh(src.geometry, MAT.shell);
      mesh.name = name;
      mesh.matrixAutoUpdate = false;
      mesh.castShadow = true;
      mesh.receiveShadow = false;
      mesh.frustumCulled = false;
      this.root.add(mesh);
      this.links.push(mesh);
    }

    // 末端夹爪：建在“工具坐标系”下（z 轴为接近方向）
    this.flangeLocal = this.makeFlangeFrame();
    this.gripper = new THREE.Group();
    this.gripper.matrixAutoUpdate = false;
    this.gripper.matrix.copy(this.flangeLocal);
    this.gripper.matrixWorldNeedsUpdate = true;
    this.links[6].add(this.gripper);
    this.buildGripper();

    this.holder = new THREE.Object3D();
    this.holder.position.set(0, 0, TOOL_OFFSET - FLANGE_OFFSET);
    this.gripper.add(this.holder);

    this.buildLivery();
    this.setJoints([0, 0, 0, 0, 0, 0]);
    return this;
  }

  /* ---- 涂装：关节灰色环带 + 基座底圈 + LEBAI 标识 ---- */
  buildLivery() {
    const J = JOINTS_MODEL; // 模型坐标
    const radial = [0.046, 0.046, 0.046, 0.0375, 0.0365, 0.0315];
    const heights = [0.007, 0.009, 0.009, 0.008, 0.008, 0.007];
    const up = new THREE.Vector3(0, 1, 0);
    const bandGeoCache = new Map();

    for (let i = 0; i < 6; i++) {
      const parent = this.links[i]; // 关节 i 的环带挂在上一级连杆上
      if (!parent) continue;
      const r = radial[i] + 0.0028;
      const key = `${r}_${heights[i]}`;
      let geo = bandGeoCache.get(key);
      if (!geo) {
        geo = new THREE.CylinderGeometry(r, r, heights[i], 56, 1, true);
        bandGeoCache.set(key, geo);
      }
      const band = new THREE.Mesh(geo, MAT.band);
      band.position.set(...J[i].p);
      const axis = new THREE.Vector3(...J[i].a).normalize();
      band.quaternion.setFromUnitVectors(up, axis);
      band.castShadow = false;
      band.receiveShadow = false;
      parent.add(band);
    }

    // 基座底圈
    const ring = new THREE.Mesh(new THREE.CylinderGeometry(0.0722, 0.0722, 0.012, 64, 1, true), MAT.band);
    ring.position.set(0, 0, 0.008);
    ring.castShadow = true;
    this.links[0].add(ring);

    // 基座底部状态灯环（呼吸灯效）
    const led = new THREE.Mesh(new THREE.CylinderGeometry(0.0726, 0.0726, 0.0022, 64, 1, true), MAT.led);
    led.position.set(0, 0, 0.0016);
    this.links[0].add(led);

    // LEBAI 标识（按工作位摆放，便于选择朝向镜头的一侧）
    this.setJoints(POSE_READY.map((d) => (d * Math.PI) / 180));
    const logoMat = this.makeLogoMaterial();
    this.addLogo(this.links[2], new THREE.Vector3(...J[1].p), new THREE.Vector3(...J[2].p), 0.080, logoMat, 0.22);
    this.addLogo(this.links[3], new THREE.Vector3(...J[2].p), new THREE.Vector3(...J[3].p), 0.086, logoMat, 0.58);
  }

  makeLogoMaterial() {
    const c = document.createElement('canvas');
    c.width = 800; c.height = 180;
    const g = c.getContext('2d');
    g.clearRect(0, 0, c.width, c.height);
    // 官方标识：深灰 LEBAI 字 + 红色图形字
    const red = '#d32027';
    const ink = '#2b3038';
    g.textBaseline = 'middle';
    g.fillStyle = ink;
    g.font = '900 132px "Arial Black", Arial, sans-serif';
    g.fillText('LEBAI', 6, 96);
    g.fillStyle = red;
    g.font = '800 104px "PingFang SC", "Microsoft YaHei", sans-serif';
    g.fillText('乐白', 468, 100);
    // 品牌环形标记
    g.strokeStyle = red;
    g.lineWidth = 15;
    g.beginPath();
    g.arc(700, 96, 36, Math.PI * 0.35, Math.PI * 1.95);
    g.stroke();
    g.beginPath();
    g.moveTo(730, 66);
    g.lineTo(742, 30);
    g.stroke();

    const tex = new THREE.CanvasTexture(c);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.anisotropy = 8;
    return new THREE.MeshStandardMaterial({
      map: tex, transparent: true, roughness: 0.42, metalness: 0.05,
      depthWrite: false, polygonOffset: true, polygonOffsetFactor: -4, polygonOffsetUnits: -4,
    });
  }

  /** 用射线打到连杆真实表面，把贴标放在命中点，保证贴合不穿模 */
  addLogo(mesh, pStart, pEnd, width, mat, t = 0.5) {
    if (!mesh) return;
    const dLocal = new THREE.Vector3().subVectors(pEnd, pStart).normalize();
    const midLocal = new THREE.Vector3().lerpVectors(pStart, pEnd, t);
    mesh.updateMatrixWorld(true);

    const midWorld = midLocal.clone().applyMatrix4(mesh.matrixWorld);
    const dWorld = dLocal.clone().transformDirection(mesh.matrixWorld).normalize();
    const camDir = new THREE.Vector3(0.55, 0.78, 0.3).normalize();

    const hits = [];
    const up = new THREE.Vector3(0, 0, 1);
    let ref = new THREE.Vector3().crossVectors(dWorld, up);
    if (ref.lengthSq() < 1e-6) ref = new THREE.Vector3().crossVectors(dWorld, new THREE.Vector3(1, 0, 0));
    ref.normalize();

    for (let i = 0; i < 36; i++) {
      const dirOut = ref.clone().applyAxisAngle(dWorld, (i / 36) * Math.PI * 2).normalize();
      const origin = midWorld.clone().addScaledVector(dirOut, 0.6);
      const ray = new THREE.Raycaster(origin, dirOut.clone().negate(), 0, 1.2);
      const hit = ray.intersectObject(mesh, false)[0];
      if (!hit || !hit.face) continue;
      const nWorld = hit.face.normal.clone().transformDirection(mesh.matrixWorld).normalize();
      if (nWorld.dot(dirOut) < 0.4) continue;
      if (Math.abs(nWorld.dot(dWorld)) > 0.5) continue;
      hits.push({ score: nWorld.dot(camDir), point: hit.point.clone(), normal: nWorld.clone() });
    }
    if (!hits.length) return;

    hits.sort((a, b) => b.score - a.score);
    const chosen = [hits[0]];
    // 再补一个朝向相反的面，正反都可见
    const back = hits.find((h) => h.normal.dot(chosen[0].normal) < -0.2);
    if (back) chosen.push(back);

    const inv = new THREE.Matrix4().copy(mesh.matrixWorld).invert();
    const h = width / 4.44;
    for (const c of chosen) {
      const pLocal = c.point.clone().applyMatrix4(inv);
      const nLocal = c.normal.clone().transformDirection(inv).normalize();
      const xAxis = dLocal.clone().addScaledVector(nLocal, -dLocal.dot(nLocal)).normalize();
      const yAxis = new THREE.Vector3().crossVectors(nLocal, xAxis).normalize();

      const plane = new THREE.Mesh(new THREE.PlaneGeometry(width, h), mat);
      plane.position.copy(pLocal).addScaledVector(nLocal, 0.0012);
      plane.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(xAxis, yAxis, nLocal));
      plane.renderOrder = 3;
      mesh.add(plane);
      this._logoCount = (this._logoCount || 0) + 1;
    }
  }

  makeFlangeFrame() {
    const j6 = this.kin.joints[5];
    const yaw = this.kin.yaw;
    // 模型坐标下的零位法兰位姿
    const z = new THREE.Vector3(-0.7071, 0.7071, 0).normalize();
    const x = new THREE.Vector3().crossVectors(z, new THREE.Vector3(0, 0, 1)).normalize();
    const y = new THREE.Vector3().crossVectors(z, x);
    const M = new THREE.Matrix4().makeBasis(x, y, z);
    const p = new THREE.Vector3(-0.0853, 0.0853, 0.85416).addScaledVector(z, FLANGE_OFFSET);
    M.setPosition(p);
    return M;
  }

  buildGripper() {
    const g = this.gripper;
    const add = (geo, mat, pos, rot) => {
      const m = new THREE.Mesh(geo, mat);
      m.castShadow = true;
      m.receiveShadow = false;
      if (pos) m.position.set(...pos);
      if (rot) m.rotation.set(...rot);
      g.add(m);
      return m;
    };
    // 法兰连接盘
    add(new THREE.CylinderGeometry(0.032, 0.032, 0.011, 40), MAT.flange, [0, 0, 0.0055], [Math.PI / 2, 0, 0]);
    // 分件线（深色金属接缝）
    add(new THREE.CylinderGeometry(0.0285, 0.0285, 0.004, 32), MAT.dark, [0, 0, 0.0125], [Math.PI / 2, 0, 0]);
    // 白色立柱（官方图中的白色圆柱连接座）
    add(new THREE.CylinderGeometry(0.028, 0.028, 0.022, 32), MAT.shell, [0, 0, 0.025], [Math.PI / 2, 0, 0]);
    // 白色圆顶本体
    const dome = add(new THREE.SphereGeometry(0.039, 40, 24), MAT.shell, [0, 0, 0.044]);
    dome.scale.set(1, 1, 0.62);

    // LMG-90 三指自定心夹爪：红色联动臂 + 灰色夹垫（对齐官方渲染图）
    const PIVOT_R = 0.03, PIVOT_Z = 0.042, ARM_L = 0.05;
    this.armParam = { r: PIVOT_R, z: PIVOT_Z, l: ARM_L };
    this.fingers = [];
    for (let i = 0; i < 3; i++) {
      const th = Math.PI / 2 + (i * Math.PI * 2) / 3; // 3 × 120° 均布
      const root = new THREE.Group();
      root.position.set(Math.cos(th) * PIVOT_R, Math.sin(th) * PIVOT_R, PIVOT_Z);
      root.rotation.z = th;
      g.add(root);

      // 联动臂绕本地 y 轴（切向）摆动
      const pivot = new THREE.Group();
      root.add(pivot);

      const arm = new THREE.Mesh(new THREE.BoxGeometry(ARM_L, 0.016, 0.015), MAT.finger);
      arm.position.set(ARM_L / 2 - 0.005, 0, 0);
      arm.castShadow = true;
      pivot.add(arm);

      // 铰链销（银色）
      const pin = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, 0.02, 20), MAT.flange);
      pivot.add(pin);

      // 夹垫组件：随臂展开后反向补偿，始终保持竖直
      const pad = new THREE.Group();
      pad.position.set(ARM_L - 0.005, 0, 0);
      pivot.add(pad);
      const bar = new THREE.Mesh(new THREE.BoxGeometry(0.009, 0.02, 0.062), MAT.finger);
      bar.castShadow = true;
      pad.add(bar);
      const face = new THREE.Mesh(new THREE.BoxGeometry(0.002, 0.018, 0.058), MAT.pad);
      face.position.set(-0.0055, 0, 0);
      pad.add(face);

      this.fingers.push({ pivot, pad });
    }
  }

  setGripper(open, immediate = false) {
    this.gripperTarget = Math.max(0.014, Math.min(0.095, open));
    if (immediate) {
      this.gripperOpen = this.gripperTarget;
      this.applyGripper();
    }
  }

  applyGripper() {
    if (!this.fingers) return;
    // 联动臂摆角 γ：使夹垫内工作面半径 = 开合的一半（自定心）
    const half = this.gripperOpen / 2;
    const g = Math.acos(THREE.MathUtils.clamp((half - 0.0235) / 0.045, -1, 1));
    for (const f of this.fingers) {
      f.pivot.rotation.y = -g; // 臂向外抬起
      f.pad.rotation.y = g;   // 反向补偿，夹垫保持竖直
    }
  }

  update(dt) {
    if (this.gripperOpen !== this.gripperTarget) {
      const d = this.gripperTarget - this.gripperOpen;
      const step = Math.sign(d) * Math.min(Math.abs(d), this.gripperRate * dt);
      this.gripperOpen += step;
      if (Math.abs(this.gripperTarget - this.gripperOpen) < 1e-4) this.gripperOpen = this.gripperTarget;
      this.applyGripper();
    }
  }

  setJoints(q) {
    this.q = q.slice();
    const frames = this.kin.linkFrames(this.q);
    for (let i = 0; i < this.links.length; i++) {
      const m = this.links[i];
      m.matrix.copy(frames[i]).multiply(this.kin.rot);
      m.matrixWorldNeedsUpdate = true;
    }
    this.root.updateMatrixWorld(true);
  }

  tcp() {
    const T = this.kin.fk(this.q);
    return T.premultiply(this.root.matrix);
  }

  tcpQuaternion() {
    return new THREE.Quaternion().setFromRotationMatrix(this.kin.fk(this.q));
  }

  localTarget(pos, yaw, tilt = 0) {
    // 世界坐标目标位姿 → 基座坐标
    const z = new THREE.Vector3(0, 0, -1).applyAxisAngle(new THREE.Vector3(1, 0, 0), tilt);
    const M = new THREE.Matrix4();
    const x = new THREE.Vector3(Math.cos(yaw), Math.sin(yaw), 0);
    x.addScaledVector(z, -x.dot(z)).normalize();
    const y = new THREE.Vector3().crossVectors(z, x);
    M.makeBasis(x, y, z);
    const p = new THREE.Vector3(...pos).sub(this.root.position);
    M.setPosition(p);
    return M;
  }

  solveTarget(pos, yaw, tilt = 0, seeds) {
    const target = this.localTarget(pos, yaw, tilt);
    const list = seeds || [this.q, ...this.seedSet(pos, yaw, tilt)];
    let best = this.kin.solveBest(target, list, this.q);
    if (!best) {
      // 二次尝试：放宽收敛阈值、更多初值
      for (const sd of list) {
        const r = this.kin.solve(target, sd, { iters: 220, lambda: 0.02 });
        if (r.posErr < 2e-3 && r.angErr < 0.02 && (!best || r.posErr < best.posErr)) best = r;
      }
    }
    return best;
  }

  /** 依次尝试多种末端朝向/高度，返回首个可行解 */
  solveFlexible(pos, yaw, { dYaw = [0, 0.5, -0.5, 1.0, -1.0], dZ = [0, 0.05, -0.04, 0.1] } = {}) {
    for (const dz of dZ) {
      for (const dy of dYaw) {
        const s = this.solveTarget(new THREE.Vector3(pos.x, pos.y, pos.z + dz), yaw + dy);
        if (s) return { sol: s, yaw: yaw + dy, z: pos.z + dz };
      }
    }
    return null;
  }

  seedSet(pos, yaw, tilt) {
    const kin = this.kin;
    const seeds = [
      [-17, 10, 93, -13, -90, -17],
      [-14, 42, 100, -56, -90, -14],
      [-40, 30, 125, -70, -90, -40],
      [10, 24, 140, -80, -90, 10],
      [-20, 60, 80, -40, -90, -20],
      [0, 26, 142, -78, -90, 0],
      [-55, 45, 110, -60, -90, -55],
      [30, 20, 130, -70, -90, 30],
      [0, 70, 60, -30, -90, 0],
    ].map((s) => s.map((d) => (d * Math.PI) / 180));
    // 按目标方位旋转基座，作为额外初值
    const base = Math.atan2(pos[1] - this.root.position.y, pos[0] - this.root.position.x) - Math.PI / 2;
    for (const s of seeds.slice(0, 3)) {
      const q = s.slice();
      q[0] = base;
      seeds.push(q);
    }
    return seeds;
  }

  async moveJ(target, duration = 1.2) {
    const q0 = this.q.slice();
    const start = performance.now();
    for (;;) {
      const t = Math.min(1, (performance.now() - start) / (duration * 1000));
      const e = easeInOut(t);
      this.setJoints(q0.map((v, i) => v + (target[i] - v) * e));
      if (t >= 1) break;
      await nextFrame();
    }
  }

  async moveL(pos, yaw, tilt = 0, duration = 0.9) {
    const startM = this.kin.fk(this.q);
    const target = this.localTarget(pos, yaw, tilt);
    const p0 = new THREE.Vector3().setFromMatrixPosition(startM);
    const p1 = new THREE.Vector3().setFromMatrixPosition(target);
    const q0 = new THREE.Quaternion().setFromRotationMatrix(startM);
    const q1 = new THREE.Quaternion().setFromRotationMatrix(target);
    const start = performance.now();
    for (;;) {
      const t = Math.min(1, (performance.now() - start) / (duration * 1000));
      const e = easeInOut(t);
      const M = new THREE.Matrix4().makeRotationFromQuaternion(q0.clone().slerp(q1, e));
      M.setPosition(p0.clone().lerp(p1, e));
      const r = this.kin.solve(M, this.q, { iters: 40 });
      if (r.posErr < 2e-3) this.setJoints(r.q);
      if (t >= 1) break;
      await nextFrame();
    }
    // 末端精修
    const r = this.kin.solve(target, this.q, { iters: 120 });
    if (r.posErr < 5e-4) this.setJoints(r.q);
  }

  attach(obj) {
    this.holder.attach(obj);
  }

  releaseTo(parent) {
    parent.attach(this.holder.children[0]);
  }
}
