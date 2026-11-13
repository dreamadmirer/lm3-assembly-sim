import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { Kinematics, TOOL_OFFSET, FLANGE_OFFSET } from './kinematics.js';
import { ROBOT_BASE, ROBOT_YAW } from './config.js';

const easeInOut = (t) => (t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2);
const nextFrame = () => new Promise((r) => requestAnimationFrame(r));

const MAT = {
  base: new THREE.MeshPhysicalMaterial({ color: 0x4b535f, roughness: 0.4, metalness: 0.45, clearcoat: 0.3 }),
  shell: new THREE.MeshPhysicalMaterial({ color: 0xf3f5f8, roughness: 0.32, metalness: 0.06, clearcoat: 0.55, clearcoatRoughness: 0.35 }),
  flange: new THREE.MeshPhysicalMaterial({ color: 0xd7dde5, roughness: 0.3, metalness: 0.55 }),
  dark: new THREE.MeshPhysicalMaterial({ color: 0x23282f, roughness: 0.42, metalness: 0.35 }),
  finger: new THREE.MeshPhysicalMaterial({ color: 0x9aa4b2, roughness: 0.28, metalness: 0.7 }),
  accent: new THREE.MeshPhysicalMaterial({ color: 0x0f9d8f, roughness: 0.35, metalness: 0.25 }),
};

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
      const mesh = new THREE.Mesh(src.geometry, name === 'ARM0' ? MAT.base : name === 'ARM6' ? MAT.flange : MAT.shell);
      mesh.name = name;
      mesh.matrixAutoUpdate = false;
      mesh.castShadow = true;
      mesh.receiveShadow = true;
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

    this.setJoints([0, 0, 0, 0, 0, 0]);
    return this;
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
      m.receiveShadow = true;
      if (pos) m.position.set(...pos);
      if (rot) m.rotation.set(...rot);
      g.add(m);
      return m;
    };
    // 法连接盘
    add(new THREE.CylinderGeometry(0.032, 0.032, 0.011, 40), MAT.flange, [0, 0, 0.0055], [Math.PI / 2, 0, 0]);
    // 夹爪本体（LMG-90 造型）
    add(new THREE.BoxGeometry(0.082, 0.056, 0.048), MAT.dark, [0, 0, 0.036]);
    add(new THREE.BoxGeometry(0.088, 0.016, 0.014), MAT.accent, [0, 0, 0.064]);
    // 导轨
    add(new THREE.BoxGeometry(0.094, 0.026, 0.008), MAT.finger, [0, 0, 0.070]);
    // 两指
    this.fingers = [];
    for (const s of [-1, 1]) {
      const f = new THREE.Mesh(new THREE.BoxGeometry(0.011, 0.030, 0.062), MAT.finger);
      f.castShadow = true;
      f.receiveShadow = true;
      f.position.set(s * 0.045, 0, 0.104);
      g.add(f);
      this.fingers.push({ mesh: f, side: s });
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
    for (const { mesh, side } of this.fingers) {
      mesh.position.x = side * (this.gripperOpen / 2 + 0.0055);
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
