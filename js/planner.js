import * as THREE from 'three';

/* 任务规划：把「抓取-搬运-放置」拆成可展示、可编辑的路径点与指令序列 */

export const DEFAULT_PARAMS = {
  approachH: 0.13,   // 预抓取高度（零件上方）
  graspRatio: 0.52,  // 抓取点高度 = 零件高度 × 比例
  transitZ: 0.27,    // 转运高度（绝对高度）
  placeZ: 0.06,      // 放置点距料盒底高度
  retreatH: 0.12,    // 放置后退回高度
  tJoint: 1.2,       // 关节空间运动时长 s
  tLinear: 0.55,     // 直线运动时长 s
  openGap: 0.012,    // 夹爪开口 = 零件宽度 + 该值
};

export const PARAM_META = {
  APPROACH_H: { key: 'approachH', min: 0.05, max: 0.3 },
  GRASP_RATIO: { key: 'graspRatio', min: 0.3, max: 0.8 },
  TRANSIT_Z: { key: 'transitZ', min: 0.18, max: 0.45 },
  PLACE_Z: { key: 'placeZ', min: 0.02, max: 0.15 },
  RETREAT_H: { key: 'retreatH', min: 0.05, max: 0.2 },
  T_JOINT: { key: 'tJoint', min: 0.4, max: 3 },
  T_LINEAR: { key: 'tLinear', min: 0.2, max: 1.5 },
};

const LABELS = {
  approach: '预抓取', grasp: '抓取点', lift: '提升', transit: '转运点', place: '放置点', retreat: '退回',
};

const COLORS = {
  approach: 0x0ea5e9, grasp: 0x14b8a6, lift: 0x22c55e, transit: 0xf59e0b, place: 0x8b5cf6, retreat: 0x64748b,
};

export const STEP_COLORS = COLORS;

/* 默认指令序列（可在代码面板中编辑） */
export const DEFAULT_STEPS = [
  { cmd: 'move_j', target: 'approach' },
  { cmd: 'move_l', target: 'grasp' },
  { cmd: 'gripper', arg: 'close' },
  { cmd: 'move_l', target: 'lift' },
  { cmd: 'move_j', target: 'transit' },
  { cmd: 'move_l', target: 'place' },
  { cmd: 'gripper', arg: 'open' },
  { cmd: 'move_j', target: 'retreat' },
  { cmd: 'home' },
];

export function graspWidth(cfg) {
  switch (cfg.type) {
    case 'cube': return cfg.size[0];
    case 'cylinder': return cfg.radius * 2;
    case 'hex': return cfg.radius * Math.sqrt(3);
    case 'tri': return cfg.radius * 1.55;
    case 'sphere': return cfg.radius * 2;
    case 'lblock': return cfg.size[0] * 0.72;
    default: return 0.06;
  }
}

function target(robot, pos, yaw, opts = {}) {
  const sol = opts.flexible ? robot.solveFlexible(pos, yaw, opts.flexible) : robot.solveTarget(pos, yaw);
  if (!sol) return null;
  if (opts.flexible) return { pos: new THREE.Vector3(pos.x, pos.y, sol.z), yaw: sol.yaw, q: sol.sol.q };
  return { pos: pos.clone(), yaw, q: sol.q };
}

/**
 * 生成一次抓取放置的完整规划
 * @returns {{targets, steps, waypoints, params, width, error?}}
 */
export function planTask({ obj, bin, slot, params: p = DEFAULT_PARAMS, robot, steps }) {
  const cfg = obj.userData.cfg;
  const h = obj.userData.height;
  const yaw = obj.rotation.z;
  const gx = obj.position.x;
  const gy = obj.position.y;

  const graspZ = Math.max(0.012, h * p.graspRatio);
  const approachZ = Math.min(h + p.approachH, 0.42);
  const px = bin.pos[0] + slot.dx;
  const py = bin.pos[1] + slot.dy;
  const placeZ = h * 0.5 + p.placeZ;
  const pYaw0 = yaw + (slot.yaw || 0);

  const approach = target(robot, new THREE.Vector3(gx, gy, approachZ), yaw);
  if (!approach) return { error: '预抓取位姿不可达' };

  const grasp = target(robot, new THREE.Vector3(gx, gy, graspZ), yaw);
  if (!grasp) return { error: '抓取位姿不可达' };

  const lift = target(robot, new THREE.Vector3(gx, gy, approachZ), yaw) || approach;

  const transit = target(robot, new THREE.Vector3(px, py, p.transitZ), pYaw0, {
    flexible: { dYaw: [0, 0.5, -0.5, 1.0, -1.0], dZ: [0, 0.05, -0.04, 0.1] },
  });
  if (!transit) return { error: '搬运位姿不可达' };

  const place = target(robot, new THREE.Vector3(px, py, placeZ), transit.yaw, {
    flexible: { dYaw: [0], dZ: [0, 0.02] },
  });
  if (!place) return { error: '放置位姿不可达' };

  let retreat = target(robot, new THREE.Vector3(px, py, place.z + p.retreatH), place.yaw, {
    flexible: { dYaw: [0, 0.5, -0.5], dZ: [0, 0.03, 0.06] },
  });
  if (!retreat) retreat = transit;

  const targets = { approach, grasp, lift, transit, place, retreat };
  const waypoints = ['approach', 'grasp', 'transit', 'place', 'retreat'].map((k) => ({
    key: k, label: LABELS[k], color: COLORS[k], pos: targets[k].pos.clone(),
  }));

  return {
    obj, binId: bin.id, cfg, h, yaw,
    width: graspWidth(cfg),
    params: { ...p },
    targets, waypoints, steps: (steps && steps.length ? steps : DEFAULT_STEPS).map((s) => ({ ...s })),
  };
}

/* ---------------- 程序文本 ---------------- */
export function programFor(plan) {
  const p = plan.params;
  const name = plan.cfg.name;
  const bin = plan.binId;
  const f = (v) => Number(v).toFixed(2);
  return `# ${name} → ${bin} 盒 · 改参数或指令后点「运行」
APPROACH_H = ${f(p.approachH)}   # 接近高度 m
GRASP_RATIO = ${f(p.graspRatio)}  # 抓取比例
TRANSIT_Z = ${f(p.transitZ)}     # 转运高度
PLACE_Z = ${f(p.placeZ)}       # 放置高度
RETREAT_H = ${f(p.retreatH)}     # 退回高度
T_JOINT = ${f(p.tJoint)}      # 关节时长 s
T_LINEAR = ${f(p.tLinear)}      # 直线时长 s

move_j approach
move_l grasp
gripper close
move_l lift
move_j transit
move_l place
gripper open
move_j retreat
home`;
}

export function parseProgram(text) {
  const params = { ...DEFAULT_PARAMS };
  const steps = [];
  const errors = [];
  const lines = text.split(/\r?\n/);

  lines.forEach((raw, i) => {
    const line = raw.replace(/#.*$/, '').trim();
    if (!line) return;
    const assign = line.match(/^([A-Z][A-Z0-9_]*)\s*=\s*(-?\d+(?:\.\d+)?)\s*$/);
    if (assign) {
      const meta = PARAM_META[assign[1]];
      if (!meta) {
        errors.push(`第 ${i + 1} 行：未知参数 ${assign[1]}`);
        return;
      }
      const v = parseFloat(assign[2]);
      if (v < meta.min || v > meta.max) {
        errors.push(`第 ${i + 1} 行：${assign[1]} 应在 ${meta.min}~${meta.max} 之间`);
        return;
      }
      params[meta.key] = v;
      return;
    }
    const parts = line.split(/\s+/);
    const cmd = parts[0];
    if (['move_j', 'move_l'].includes(cmd)) {
      const t = parts[1];
      if (!LABELS[t]) { errors.push(`第 ${i + 1} 行：未知路径点 ${t || '(空)'}`); return; }
      steps.push({ cmd, target: t });
    } else if (cmd === 'gripper') {
      const arg = parts[1];
      if (!['open', 'close'].includes(arg)) { errors.push(`第 ${i + 1} 行：gripper 需为 open/close`); return; }
      steps.push({ cmd, arg });
    } else if (cmd === 'home') {
      steps.push({ cmd: 'home' });
    } else if (cmd === 'wait') {
      const s = parseFloat(parts[1]);
      if (!Number.isFinite(s) || s < 0 || s > 5) { errors.push(`第 ${i + 1} 行：wait 时间无效`); return; }
      steps.push({ cmd: 'wait', seconds: s });
    } else {
      errors.push(`第 ${i + 1} 行：未知指令 ${cmd}`);
    }
  });

  if (!steps.length) errors.push('没有任何可执行指令');
  return { params, steps, errors };
}

export function planLength(waypoints) {
  let len = 0;
  for (let i = 0; i < waypoints.length - 1; i++) len += waypoints[i].pos.distanceTo(waypoints[i + 1].pos);
  return len;
}
