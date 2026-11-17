import * as THREE from 'three';
import { JOINT_LIMITS } from './config.js';

/**
 * 乐白官方 Python SDK（lebai_sdk）的浏览器仿真层。
 *
 * 对齐要点（详见 docs/SDK对齐说明.md）：
 *  - init / connect / start_sys / stop_sys 生命周期
 *  - get_robot_state：IDLE / MOVING / ESTOP
 *  - get_kin_data：actual_joint_pose（rad）、actual_tcp_pose（{x,y,z,rx,ry,rz}，m/rad）
 *  - movej(pose, a, v, t, r) / movel(pose, a, v, t, r) 返回 motion_id；t>0 时忽略 a、v
 *  - wait_move(id?) / stop_move() / pause_move() / resume_move() / get_motion_state(id)
 *  - kinematics_forward / kinematics_inverse / pose_trans / pose_add
 *  - set_claw(force, amplitude) / get_claw()
 */

export const ROBOT_STATE = { IDLE: 'IDLE', MOVING: 'MOVING', ESTOP: 'ESTOP' };
export const MOTION_STATE = { WAIT: 'WAIT', RUNNING: 'RUNNING', FINISHED: 'FINISHED' };

export class LebaiError extends Error {
  constructor(code, message) {
    super(`[${code}] ${message}`);
    this.name = 'LebaiError';
    this.code = code;
    this.msg = message;
  }
}

const D2R = Math.PI / 180;
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

/** {x,y,z,rx,ry,rz}（m / rad，R = Rz·Ry·Rx）→ 基座系位姿矩阵 */
export function poseToMatrix(p) {
  const m = new THREE.Matrix4();
  const q = new THREE.Quaternion().multiplyQuaternions(
    new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), p.rz || 0),
    new THREE.Quaternion().multiplyQuaternions(
      new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), p.ry || 0),
      new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), p.rx || 0)
    )
  );
  m.makeRotationFromQuaternion(q);
  m.setPosition(p.x, p.y, p.z);
  return m;
}

/** 位姿矩阵 → {x,y,z,rx,ry,rz}（与 poseToMatrix 互逆，ZYX 欧拉角） */
export function matrixToPose(m) {
  const e = m.elements;
  const rx = Math.atan2(e[6], e[10]);
  const ry = Math.asin(clamp(-e[2], -1, 1));
  const rz = Math.atan2(e[1], e[0]);
  return { x: e[12], y: e[13], z: e[14], rx, ry, rz };
}

export class LebaiSim {
  constructor(robot, app) {
    this.robot = robot;
    this.app = app;
    this.connected = false;
    this.ip = '';
    this.state = ROBOT_STATE.MOVING; // 上电初始：未 start_sys
    this.motions = new Map();
    this.motionPromises = new Map();
    this.nextId = 1;
    this._chain = Promise.resolve();
    this._paused = false;
    this.claw = { force: 50, amplitude: 100, weight: 0, hold_on: true };
    this.grasped = null;
    this.lastError = null;
    this.onStateChange = null;
  }

  /* ---------------- 生命周期 ---------------- */
  init() {
    this.log('lebai_sdk.init() · 仿真 SDK 已初始化');
  }

  connect(ip = '192.168.0.100') {
    this.ip = ip;
    this.connected = true;
    this.state = ROBOT_STATE.IDLE;
    this.log(`connect(${ip}) · 已建立控制器连接（仿真）`);
    this.emit();
    return this;
  }

  start_sys() {
    this.requireConnection();
    if (this.state === ROBOT_STATE.ESTOP) throw new LebaiError(1003, '机器人处于急停状态，请先复位');
    this.state = ROBOT_STATE.IDLE;
    this.log('start_sys() · 手臂已启动');
    this.emit();
  }

  stop_sys() {
    this.robot.stopMotion();
    this.state = ROBOT_STATE.IDLE;
    this.log('stop_sys() · 手臂已停止');
    this.emit();
  }

  get_robot_state() {
    this.requireConnection();
    return this.state;
  }

  /** 急停（UI 急停按钮 / 安全回路） */
  estop() {
    this.robot.stopMotion();
    this.state = ROBOT_STATE.ESTOP;
    this.log('ESTOP · 急停触发，运动已中断', 'warn');
    this.emit();
  }

  /* ---------------- 状态反馈 ---------------- */
  get_kin_data() {
    this.requireConnection();
    const T = this.robot.kin.fk(this.robot.q); // 基座系
    const pose = matrixToPose(T);
    return {
      actual_joint_pose: this.robot.q.slice(),
      actual_tcp_pose: pose,
      target_joint_pose: this.robot.q.slice(),
      target_tcp_pose: pose,
      flange_pose: pose,
    };
  }

  get_motion_state(id) {
    return this.motions.get(id) || MOTION_STATE.FINISHED;
  }

  get_running_motion() {
    for (const [id, st] of this.motions) if (st !== MOTION_STATE.FINISHED) return id;
    return 0;
  }

  /* ---------------- 运动控制 ---------------- */
  movej(pose, a = 0.5, v = 0.2, t = 0, r = 0) {
    this.requireMotionReady('movej');
    const q = this.resolveJointTarget(pose);
    this.checkJointLimits(q);
    const dur = t > 0 ? t : this.durationJoint(q, a, v);
    return this.enqueue(`movej`, () => this.robot.moveJ(q, dur));
  }

  movel(pose, a = 0.3, v = 0.1, t = 0, r = 0) {
    this.requireMotionReady('movel');
    const M = this.resolveCartesianTarget(pose);
    // 反解校验：与实机一致，不可达 / 奇异点直接报错，而不是"走不到也不报"
    const test = this.robot.kin.solve(M, this.robot.q, { iters: 240 });
    if (test.posErr > 2e-3) throw new LebaiError(1002, '目标位姿不可达（奇异点或超出工作空间）');
    const dur = t > 0 ? t : this.durationLinear(M, a, v);
    return this.enqueue(`movel`, () => this.robot.moveLMatrix(M, dur));
  }

  wait_move(id) {
    if (id != null && this.motionPromises.has(id)) return this.motionPromises.get(id);
    return this._chain;
  }

  stop_move() {
    this.robot.stopMotion();
    this.log('stop_move() · 已停止运动（保持当前位置）', 'warn');
  }

  pause_move() {
    this._paused = true;
    this.log('pause_move() · 暂停');
  }

  resume_move() {
    this._paused = false;
    this.log('resume_move() · 恢复');
  }

  /* ---------------- 运动学 ---------------- */
  kinematics_forward(jointPose) {
    const q = jointPose.map(Number);
    const T = this.robot.kin.fk(q);
    return matrixToPose(T);
  }

  kinematics_inverse(cartesianPose, refJoint) {
    const M = poseToMatrix(cartesianPose);
    const seed = refJoint ? refJoint.map(Number) : this.robot.q.slice();
    const r = this.robot.kin.solve(M, seed, { iters: 200 });
    if (r.posErr > 2e-3) throw new LebaiError(1002, '目标位姿不可达（奇异点或超出工作空间）');
    return r.q;
  }

  pose_trans(base, offset) {
    return matrixToPose(poseToMatrix(base).multiply(poseToMatrix(offset)));
  }

  pose_add(base, delta, frame) {
    const Mb = poseToMatrix(base);
    const F = frame ? poseToMatrix(frame) : new THREE.Matrix4();
    const D = poseToMatrix(delta);
    const Fd = F.clone().multiply(D); // 在参考系下的偏移
    return matrixToPose(Mb.multiply(Fd));
  }

  /* ---------------- 夹爪 ---------------- */
  set_claw(force = 50, amplitude = 100) {
    const f = clamp(Number(force), 0, 100);
    const amp = clamp(Number(amplitude), 0, 100);
    const prevAmp = this.claw.amplitude;
    this.claw.force = f;
    this.claw.amplitude = amp;
    const width = 0.014 + (0.095 - 0.014) * (amp / 100);
    this.robot.setGripper(width);
    this.log(`set_claw(force=${f}, amplitude=${amp}) · 开口 ${(width * 1000).toFixed(0)} mm`);

    // 开/合由开合度变化方向决定（与实机一致：amplitude 变小 = 夹紧）
    if (amp < prevAmp - 1) {
      const obj = this.findGraspTarget();
      if (obj && !this.grasped) {
        this.robot.attach(obj);
        this.grasped = obj;
        obj.userData.busy = true;
        this.claw.hold_on = false;
        setTimeout(() => { this.claw.hold_on = true; }, 320);
        this.log(`夹爪夹紧 · ${obj.userData.cfg.name}`, 'ok');
      } else if (!obj) {
        this.claw.hold_on = true;
        this.log('夹爪闭合 · 未检测到工件', 'warn');
      }
    } else if (this.grasped) {
      this.app.releaseObject(this.grasped);
      this.grasped = null;
      this.claw.hold_on = true;
      this.log('夹爪张开 · 已释放工件');
    }
    return this.get_claw();
  }

  get_claw() {
    return {
      force: this.claw.force,
      amplitude: this.claw.amplitude,
      weight: this.grasped ? (this.grasped.userData.cfg.mass || 0.1) : 0,
      hold_on: this.claw.hold_on,
    };
  }

  /* ---------------- IO（占位，语义对齐） ---------------- */
  get_di(scope, index) { return this.di?.[`${scope}_${index}`] ?? 0; }
  set_do(scope, index, value) {
    this.di = this.di || {};
    this.di[`${scope}_${index}`] = value ? 1 : 0;
    this.log(`set_do("${scope}", ${index}, ${value})`);
  }
  get_do(scope, index) { return this.di?.[`${scope}_${index}`] ?? 0; }

  /* ---------------- 内部 ---------------- */
  requireConnection() {
    if (!this.connected) throw new LebaiError(1004, '未连接控制器，请先调用 connect()');
  }

  requireMotionReady(name) {
    this.requireConnection();
    if (this.state === ROBOT_STATE.ESTOP) throw new LebaiError(1003, `急停状态下不能执行 ${name}()`);
  }

  resolveJointTarget(pose) {
    if (Array.isArray(pose)) {
      if (pose.length !== 6) throw new LebaiError(1001, '关节位置需为长度 6 的列表');
      return pose.map(Number);
    }
    // 笛卡尔位姿也允许传给 movej（官方支持）
    return this.kinematics_inverse(pose);
  }

  resolveCartesianTarget(pose) {
    if (Array.isArray(pose)) return this.robot.kin.fk(pose.map(Number));
    return poseToMatrix(pose);
  }

  checkJointLimits(q) {
    for (let i = 0; i < 6; i++) {
      const [lo, hi] = JOINT_LIMITS[i];
      if (q[i] < lo - 1e-6 || q[i] > hi + 1e-6) {
        throw new LebaiError(1001, `J${i + 1} 目标 ${(q[i] / D2R).toFixed(1)}° 超出安全范围 [${(lo / D2R).toFixed(0)}°, ${(hi / D2R).toFixed(0)}°]`);
      }
    }
  }

  durationJoint(q, a, v) {
    let dq = 0;
    for (let i = 0; i < 6; i++) dq = Math.max(dq, Math.abs(q[i] - this.robot.q[i]));
    const vv = Math.max(0.05, v);
    const aa = Math.max(0.1, a);
    return clamp(dq / vv + vv / aa, 0.2, 8);
  }

  durationLinear(M, a, v) {
    const T0 = this.robot.kin.fk(this.robot.q);
    const p0 = new THREE.Vector3().setFromMatrixPosition(T0);
    const p1 = new THREE.Vector3().setFromMatrixPosition(M);
    const q0 = new THREE.Quaternion().setFromRotationMatrix(T0);
    const q1 = new THREE.Quaternion().setFromRotationMatrix(M);
    const dist = p0.distanceTo(p1) + 0.12 * q0.angleTo(q1);
    const vv = Math.max(0.01, v);
    const aa = Math.max(0.05, a);
    return clamp(dist / vv + vv / aa, 0.2, 8);
  }

  /** 运动队列：与实机一致，连续指令按序执行，move_xxx 立即返回 id */
  enqueue(name, run) {
    const id = this.nextId++;
    this.motions.set(id, MOTION_STATE.WAIT);
    const prev = this._chain;
    let resolveDone;
    const done = new Promise((res) => { resolveDone = res; });
    this.motionPromises.set(id, done);
    this._chain = prev.then(async () => {
      if (this.state === ROBOT_STATE.ESTOP) {
        this.motions.set(id, MOTION_STATE.FINISHED);
        resolveDone();
        return;
      }
      this.motions.set(id, MOTION_STATE.RUNNING);
      this.state = ROBOT_STATE.MOVING;
      this.emit();
      try {
        await run();
      } catch (err) {
        this.lastError = err;
        this.motions.set(id, MOTION_STATE.FINISHED);
        resolveDone();
        if (this.state !== ROBOT_STATE.ESTOP) this.state = ROBOT_STATE.IDLE;
        this.emit();
        throw err;
      }
      this.motions.set(id, MOTION_STATE.FINISHED);
      const busy = [...this.motions.values()].some((s) => s !== MOTION_STATE.FINISHED);
      if (!busy) {
        this.state = ROBOT_STATE.IDLE;
        this.emit();
      }
      resolveDone();
    });
    // 保证队列异常不阻断后续指令
    this._chain = this._chain.catch(() => {});
    return id;
  }

  findGraspTarget() {
    const tcp = new THREE.Vector3().setFromMatrixPosition(this.robot.tcp());
    // 规划阶段已确定目标工件时，优先夹它（容差放宽到 160 mm）
    if (this.graspHint) {
      const h = this.graspHint;
      const hp = new THREE.Vector3().setFromMatrixPosition(h.matrixWorld);
      if (!h.userData.inBin && Math.hypot(hp.x - tcp.x, hp.y - tcp.y) < 0.16) return h;
    }
    let best = null;
    let bestD = 0.09;
    for (const obj of this.app.world.objects) {
      if (obj.userData.inBin || obj.userData.busy) continue;
      const p = new THREE.Vector3().setFromMatrixPosition(obj.matrixWorld);
      const d = Math.hypot(p.x - tcp.x, p.y - tcp.y);
      if (d < bestD) { bestD = d; best = obj; }
    }
    return best;
  }

  /** 任务层已知目标工件时，给夹爪一个提示（不影响 SDK 语义） */
  hintGrasp(obj) {
    this.graspHint = obj;
  }

  emit() {
    if (this.onStateChange) this.onStateChange(this.state);
  }

  log(msg, kind = '') {
    if (this.app && this.app.log) this.app.log(`[SDK] ${msg}`, kind);
  }
}

export { D2R };
