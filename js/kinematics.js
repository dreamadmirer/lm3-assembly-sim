import * as THREE from 'three';
import { JOINT_LIMITS } from './config.js';

/* ---------- 由 CAD 提取的关节轴线（模型坐标，单位 m，Z 轴向上） ---------- */
const JOINTS_MODEL = [
  { p: [0.0, 0.0, 0.0975], a: [0, 0, 1] },
  { p: [-0.04596, 0.04596, 0.21583], a: [-0.7071, 0.7071, 0] },
  { p: [-0.09971, 0.09971, 0.49583], a: [-0.7071, 0.7071, 0] },
  { p: [-0.01577, 0.01577, 0.75583], a: [-0.7071, 0.7071, 0] },
  { p: [-0.08530, 0.08530, 0.75583], a: [0, 0, 1] },
  { p: [-0.08530, 0.08530, 0.85416], a: [-0.7071, 0.7071, 0] },
];

export const FLANGE_OFFSET = 0.083; // J6 轴线原点到法兰面
export const TOOL_OFFSET = 0.173;   // J6 轴线原点到两指中心（TCP）

const V = (x, y, z) => new THREE.Vector3(x, y, z);

function expScrew(p, axis, theta) {
  const R = new THREE.Matrix4().makeRotationAxis(axis, theta);
  const rp = p.clone().applyMatrix4(R);
  R.setPosition(p.x - rp.x, p.y - rp.y, p.z - rp.z);
  return R;
}

function solveLinear6(A, b) {
  // 高斯消元（6×6，带部分主元）
  const n = 6;
  const M = A.map((row, i) => row.concat([b[i]]));
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r;
    if (Math.abs(M[piv][col]) < 1e-12) return null;
    if (piv !== col) { const t = M[piv]; M[piv] = M[col]; M[col] = t; }
    const d = M[col][col];
    for (let c = col; c <= n; c++) M[col][c] /= d;
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = M[r][col];
      if (!f) continue;
      for (let c = col; c <= n; c++) M[r][c] -= f * M[col][c];
    }
  }
  return M.map((row) => row[n]);
}

export class Kinematics {
  constructor(yaw) {
    const Ryaw = new THREE.Matrix4().makeRotationZ(yaw);
    this.yaw = yaw;
    this.rot = Ryaw.clone();
    this.joints = JOINTS_MODEL.map((j) => ({
      p: V(...j.p).applyMatrix4(Ryaw),
      a: V(...j.a).normalize().transformDirection(Ryaw),
    }));

    const z = this.joints[5].a.clone();
    const x = new THREE.Vector3().crossVectors(z, V(0, 0, 1)).normalize();
    const y = new THREE.Vector3().crossVectors(z, x).normalize();
    // 零位 TCP 位姿（模型坐标）
    const origin = this.joints[5].p.clone().addScaledVector(z, TOOL_OFFSET);
    const M = new THREE.Matrix4().makeBasis(x, y, z);
    M.setPosition(origin);
    this.M = M;
    // 法兰坐标系（零位）
    const O = this.joints[5].p.clone().addScaledVector(z, FLANGE_OFFSET);
    const F = new THREE.Matrix4().makeBasis(x, y, z);
    F.setPosition(O);
    this.Mflange = F;

    this.limits = JOINT_LIMITS;
  }

  /** 世界（基座）坐标下的 TCP 位姿 */
  fk(q) {
    const T = new THREE.Matrix4();
    for (let i = 0; i < 6; i++) T.multiply(expScrew(this.joints[i].p, this.joints[i].a, q[i]));
    return T.multiply(this.M);
  }

  /** 各连杆在基座坐标下的位姿（连杆 i 受关节 0..i-1 驱动） */
  linkFrames(q) {
    const frames = [];
    let T = new THREE.Matrix4();
    frames.push(T.clone());
    for (let i = 0; i < 6; i++) {
      T = T.clone().multiply(expScrew(this.joints[i].p, this.joints[i].a, q[i]));
      frames.push(T.clone()); // frames[i+1] = 连杆 i+1 的位姿
    }
    return frames;
  }

  jacobian(q, frames, tcp) {
    const J = [];
    for (let i = 0; i < 6; i++) {
      const w = this.joints[i].a.clone().transformDirection(frames[i]);
      const p = this.joints[i].p.clone().applyMatrix4(frames[i]);
      const v = new THREE.Vector3().crossVectors(w, tcp.clone().sub(p));
      J.push([v.x, v.y, v.z, w.x, w.y, w.z]);
    }
    // 转置为 6×6 列主序：J[行][列]
    const M = Array.from({ length: 6 }, () => new Array(6).fill(0));
    for (let c = 0; c < 6; c++) for (let r = 0; r < 6; r++) M[r][c] = J[c][r];
    return M;
  }

  solve(target, seed, { iters = 90, lambda = 0.035, maxStep = 0.25 } = {}) {
    const q = seed.slice();
    const tp = new THREE.Vector3().setFromMatrixPosition(target);
    const tq = new THREE.Quaternion().setFromRotationMatrix(target);
    let best = { q: q.slice(), posErr: Infinity, angErr: Infinity };

    for (let it = 0; it < iters; it++) {
      const T = this.fk(q);
      const cp = new THREE.Vector3().setFromMatrixPosition(T);
      const cq = new THREE.Quaternion().setFromRotationMatrix(T);
      const ep = tp.clone().sub(cp);
      const qe = tq.clone().multiply(cq.clone().invert());
      if (qe.w < 0) qe.set(-qe.x, -qe.y, -qe.z, -qe.w);
      const s = Math.sqrt(Math.max(0, 1 - qe.w * qe.w));
      const angle = 2 * Math.acos(Math.min(1, Math.abs(qe.w)));
      const eo = s > 1e-6
        ? new THREE.Vector3(qe.x / s, qe.y / s, qe.z / s).multiplyScalar(angle)
        : new THREE.Vector3();

      const perr = ep.length();
      if (perr < best.posErr) best = { q: q.slice(), posErr: perr, angErr: angle };
      if (perr < 8e-5 && angle < 8e-4) break;

      const frames = this.linkFrames(q);
      const J = this.jacobian(q, frames, cp);

      const e = [ep.x, ep.y, ep.z, eo.x * 0.35, eo.y * 0.35, eo.z * 0.35];
      const A = Array.from({ length: 6 }, (_, r) =>
        Array.from({ length: 6 }, (_, c) => {
          let sum = 0;
          for (let k = 0; k < 6; k++) sum += J[r][k] * J[c][k];
          return sum + (r === c ? lambda * lambda : 0);
        })
      );
      const y = solveLinear6(A, e);
      if (!y) break;
      const dq = new Array(6).fill(0);
      for (let i = 0; i < 6; i++) {
        let sum = 0;
        for (let k = 0; k < 6; k++) sum += J[k][i] * y[k];
        dq[i] = sum;
      }
      let m = 0;
      for (let i = 0; i < 6; i++) m = Math.max(m, Math.abs(dq[i]));
      const scale = m > maxStep ? maxStep / m : 1;
      for (let i = 0; i < 6; i++) {
        const lim = this.limits[i];
        q[i] = Math.min(lim[1], Math.max(lim[0], q[i] + dq[i] * scale));
      }
    }
    return best;
  }

  /** 多初值求解，返回可行解（按连续性/姿态代价择优） */
  solveBest(target, seeds, prefer) {
    let best = null;
    for (const seed of seeds) {
      const r = this.solve(target, seed);
      if (r.posErr > 1e-3 || r.angErr > 0.01) continue;
      let cost = 0;
      if (prefer) for (let i = 0; i < 6; i++) cost += Math.abs(r.q[i] - prefer[i]);
      else for (let i = 0; i < 6; i++) cost += Math.abs(r.q[i]) * 0.2;
      if (!best || cost < best.cost) best = { ...r, cost };
    }
    return best;
  }

  wrap(q) {
    const out = [];
    for (let i = 0; i < 6; i++) {
      let v = q[i];
      while (v > Math.PI) v -= 2 * Math.PI;
      while (v < -Math.PI) v += 2 * Math.PI;
      out.push(v);
    }
    return out;
  }
}

/** 生成给定位置 + 工具指向的位姿矩阵；toolZ 为工具接近方向，yaw 为两指开合方向 */
export function poseMatrix(pos, toolZ, yaw) {
  const z = new THREE.Vector3(...toolZ).normalize();
  const x = new THREE.Vector3(Math.cos(yaw), Math.sin(yaw), 0);
  x.addScaledVector(z, -x.dot(z));
  if (x.lengthSq() < 1e-8) x.set(1, 0, 0);
  x.normalize();
  const y = new THREE.Vector3().crossVectors(z, x);
  const M = new THREE.Matrix4().makeBasis(x, y, z);
  M.setPosition(new THREE.Vector3(...pos));
  return M;
}
