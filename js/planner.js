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
  gripW: null,       // 夹持宽度（null 表示按零件自动计算）
};

export const PARAM_META = {
  APPROACH_H: { key: 'approachH', min: 0.05, max: 0.3 },
  GRASP_RATIO: { key: 'graspRatio', min: 0.3, max: 0.8 },
  TRANSIT_Z: { key: 'transitZ', min: 0.18, max: 0.45 },
  PLACE_Z: { key: 'placeZ', min: 0.02, max: 0.15 },
  RETREAT_H: { key: 'retreatH', min: 0.05, max: 0.2 },
  T_JOINT: { key: 'tJoint', min: 0.4, max: 3 },
  T_LINEAR: { key: 'tLinear', min: 0.2, max: 1.5 },
  GRIP_W: { key: 'gripW', min: 0.012, max: 0.094 },
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

/* ---------------- 程序文本（Python） ---------------- */
const WP_ORDER = ['approach', 'grasp', 'lift', 'transit', 'place', 'retreat'];

const STEP_NOTE = {
  approach: '预抓取',
  grasp: '下降到抓取点',
  lift: '提升',
  transit: '转运',
  place: '放置点',
  retreat: '退回',
};

/** 弧度 → 角度，并归一化到 (-180, 180] */
function pyDeg(rad) {
  const d = (rad * 180) / Math.PI;
  return ((((d + 180) % 360) + 360) % 360) - 180;
}

const f3 = (v) => Number(v).toFixed(3);
const f2 = (v) => Number(v).toFixed(2);
const f1 = (v) => Number(v).toFixed(1);

function stepLine(code, note) {
  return `    ${code.padEnd(40)}# ${note}`;
}

/**
 * 生成可读的 Python 任务程序
 * plan 可以只带 {cfg, binId, params}（此时没有解算位姿，仅输出骨架）
 */
export function programFor(plan) {
  const p = { ...DEFAULT_PARAMS, ...(plan.params || {}) };
  const name = (plan.cfg && plan.cfg.name) || '零件';
  const bin = plan.binId || 'A';
  const targets = plan.targets || null;
  const width = plan.width != null ? plan.width : (p.gripW || 0.07);
  const steps = (plan.steps && plan.steps.length ? plan.steps : DEFAULT_STEPS);

  if (!targets) {
    return `# -*- coding: utf-8 -*-
"""LM3 抓取放置任务 · ${name} → ${bin} 盒"""

# 当前零件位姿超出工作空间，无法解算路径点
# 可尝试：减小 APPROACH_H / TRANSIT_Z，或先移动零件
raise RuntimeError("target pose unreachable")
`;
  }

  const key = (k) => `${JSON.stringify(k)}:`.padEnd(14);
  const wpRows = [];
  const qRows = [];
  for (const k of WP_ORDER) {
    const t = targets[k];
    if (!t) continue;
    const rz = f1(pyDeg(t.yaw));
    wpRows.push(`  ${key(k)}(${f3(t.pos.x)}, ${f3(t.pos.y)}, ${f3(t.pos.z)}, 180.0, 0.0, ${rz}),`);
    qRows.push(`  ${key(k)}(${t.q.map((v) => f1(pyDeg(v))).join(', ')}),`);
  }

  let n = 0;
  const body = steps.map((s) => {
    n += 1;
    if (s.cmd === 'move_j') {
      return stepLine(`robot.movej(Q[${JSON.stringify(s.target)}], T_JOINT)`, `${n} ${STEP_NOTE[s.target] || ''}`.trim());
    }
    if (s.cmd === 'move_l') {
      return stepLine(`robot.movel(WP[${JSON.stringify(s.target)}], T_LINEAR)`, `${n} ${STEP_NOTE[s.target] || ''}`.trim());
    }
    if (s.cmd === 'gripper') {
      const close = s.arg === 'close';
      return stepLine(`robot.gripper_${close ? 'close' : 'open'}()`, `${n} ${close ? '夹紧零件' : '松开工件'}`);
    }
    if (s.cmd === 'home') return stepLine('robot.home()', `${n} 回工作位`);
    if (s.cmd === 'wait') return stepLine(`time.sleep(${f2(s.seconds)})`, `${n} 等待`);
    return stepLine('# 未知指令', '');
  }).join('\n');

  return `# -*- coding: utf-8 -*-
"""LM3 抓取放置任务 · ${name} → ${bin} 盒
由仿真平台按当前场景解算生成；修改参数或指令后点「运行」即可复现
"""
import time
from lebai import LebaiRobot

ROBOT_IP    = "192.168.0.100"   # 控制器 IP
APPROACH_H  = ${f3(p.approachH)}   # 预抓取高度 m（零件上方）
GRASP_RATIO = ${f3(p.graspRatio)}   # 抓取高度 = 零件高 × 比例
TRANSIT_Z   = ${f3(p.transitZ)}   # 转运高度 m
PLACE_Z     = ${f3(p.placeZ)}   # 放置点距盒底 m
RETREAT_H   = ${f3(p.retreatH)}   # 退回高度 m
T_JOINT     = ${f3(p.tJoint)}   # 关节运动时长 s
T_LINEAR    = ${f3(p.tLinear)}   # 直线运动时长 s
GRIP_W      = ${f3(width)}   # 夹持宽度 m

# 路径点 TCP 位姿：[x, y, z, rx, ry, rz]，单位 m / deg
WP = {
${wpRows.join('\n')}
}

# 各路径点对应关节角：[J1, J2, J3, J4, J5, J6]，单位 deg
Q = {
${qRows.join('\n')}
}

def main():
    robot = LebaiRobot(ROBOT_IP)
    robot.login()          # 上电并接管控制权
    robot.start_task()     # 进入运动任务模式
    robot.set_payload(3.0) # 末端负载 kg

${body}

    robot.stop_task()
    robot.logout()         # 释放控制权

if __name__ == "__main__":
    main()
`;
}

/* ---------------- 程序解析（Python 子集） ---------------- */
const KNOWN_KEYS = Object.keys(LABELS);

const NOOP_METHODS = new Set([
  'login', 'logout', 'start_task', 'stop_task', 'print', 'main',
  'connect', 'disconnect', 'enable', 'disable', 'set_payload', 'set_speed',
  'wait_until', 'reset',
]);

function stripComment(line) {
  let quote = null;
  for (let i = 0; i < line.length; i++) {
    const c = line[i];
    if (quote) {
      if (c === quote && line[i - 1] !== '\\') quote = null;
      continue;
    }
    if (c === '"' || c === "'") { quote = c; continue; }
    if (c === '#') return line.slice(0, i);
  }
  return line;
}

function waypointKey(args) {
  const quoted = args.match(/["']([A-Za-z_]\w*)["']/);
  if (quoted) return quoted[1].toLowerCase();
  const named = args.match(/\b(?:Q|P|WP|POSE|JOINT)_?([A-Za-z]\w*)\b/i);
  if (named) return named[1].toLowerCase();
  return null;
}

function lastNumber(args) {
  const parts = args.split(',');
  const last = parts[parts.length - 1].trim();
  return /^-?\d+(?:\.\d+)?$/.test(last) ? parseFloat(last) : null;
}

export function parseProgram(text) {
  const params = { ...DEFAULT_PARAMS };
  const steps = [];
  const errors = [];
  const lines = String(text).split(/\r?\n/);
  let depth = 0;   // 花括号/方括号块内
  let doc = false; // 三引号字符串内

  lines.forEach((raw, i) => {
    const no = i + 1;
    let line = raw;

    const triple = line.match(/"""|'''/);
    if (doc) {
      if (triple) doc = false;
      return;
    }
    if (triple) {
      const count = (line.match(/"""|'''/g) || []).length;
      if (count === 1) doc = true;
      return;
    }

    line = stripComment(line);
    if (!line.trim()) return;
    line = line.trim();

    const opens = (line.match(/[{[]/g) || []).length;
    const closes = (line.match(/[}\]]/g) || []).length;
    if (depth > 0) { depth = Math.max(0, depth + opens - closes); return; }
    if (opens > closes) { depth = opens - closes; return; }

    if (/^(import|from|def|class|return|pass|del|assert|global|nonlocal|yield)\b/.test(line)) return;
    if (/^(if|elif|else|for|while|try|except|finally|with|match|case)\b/.test(line)) {
      if (!/^if\s+__name__/.test(line)) errors.push(`第 ${no} 行：不支持的控制流「${line}」`);
      return;
    }

    // 常量赋值
    const assign = line.match(/^([A-Za-z_]\w*)\s*=\s*(.+)$/);
    if (assign) {
      const name = assign[1];
      const value = assign[2].trim();
      if (/^-?\d+(?:\.\d+)?$/.test(value)) {
        const meta = PARAM_META[name.toUpperCase()];
        if (!meta) {
          errors.push(`第 ${no} 行：未知参数 ${name}`);
          return;
        }
        const v = parseFloat(value);
        if (v < meta.min || v > meta.max) {
          errors.push(`第 ${no} 行：${name} 应在 ${meta.min} ~ ${meta.max} 之间`);
          return;
        }
        params[meta.key] = v;
      }
      return; // 对象绑定 / 列表赋值等，忽略
    }

    // 兼容旧式简写：move_j approach / gripper close / home / wait 0.5
    const parts = line.split(/\s+/);
    if (['move_j', 'move_l'].includes(parts[0]) && parts.length === 2) {
      const t = parts[1].toLowerCase();
      if (!KNOWN_KEYS.includes(t)) { errors.push(`第 ${no} 行：未知路径点 ${parts[1]}`); return; }
      steps.push({ cmd: parts[0], target: t });
      return;
    }
    if (parts[0] === 'gripper' && parts.length === 2) {
      if (!['open', 'close'].includes(parts[1])) { errors.push(`第 ${no} 行：gripper 需为 open / close`); return; }
      steps.push({ cmd: 'gripper', arg: parts[1] });
      return;
    }
    if (line === 'home') { steps.push({ cmd: 'home' }); return; }
    if (parts[0] === 'wait' && parts.length === 2) {
      const s = parseFloat(parts[1]);
      if (!Number.isFinite(s) || s < 0 || s > 5) { errors.push(`第 ${no} 行：等待时间无效`); return; }
      steps.push({ cmd: 'wait', seconds: s });
      return;
    }

    // 函数调用
    const call = line.match(/^(?:[A-Za-z_]\w*\s*\.\s*)*([A-Za-z_]\w*)\s*\((.*)\)\s*:?$/);
    if (!call) {
      errors.push(`第 ${no} 行：无法识别的语句「${line}」`);
      return;
    }
    const meth = call[1].toLowerCase();
    const args = call[2].trim();

    if (['movej', 'move_j', 'movel', 'move_l'].includes(meth)) {
      const cmd = meth.startsWith('movej') || meth === 'move_j' ? 'move_j' : 'move_l';
      const k = waypointKey(args);
      if (!k) { errors.push(`第 ${no} 行：${meth}() 未指定路径点`); return; }
      if (!KNOWN_KEYS.includes(k)) { errors.push(`第 ${no} 行：未知路径点 ${k}`); return; }
      const step = { cmd, target: k };
      const dur = lastNumber(args);
      if (dur !== null) step.time = dur;
      steps.push(step);
      return;
    }

    if (meth.includes('gripper')) {
      if (/open/.test(meth) || /\bopen\b/.test(args)) steps.push({ cmd: 'gripper', arg: 'open' });
      else if (/close/.test(meth) || /\bclose\b/.test(args)) steps.push({ cmd: 'gripper', arg: 'close' });
      else errors.push(`第 ${no} 行：gripper_*() 需为 open 或 close`);
      return;
    }

    if (meth === 'home' || meth === 'move_home') { steps.push({ cmd: 'home' }); return; }

    if (['sleep', 'wait', 'delay'].includes(meth)) {
      const s = parseFloat(args);
      if (!Number.isFinite(s) || s < 0 || s > 5) { errors.push(`第 ${no} 行：等待时间无效`); return; }
      steps.push({ cmd: 'wait', seconds: s });
      return;
    }

    if (NOOP_METHODS.has(meth)) return;

    errors.push(`第 ${no} 行：未知指令 ${call[1]}()`);
  });

  if (!steps.length) errors.push('程序中没有可执行的指令');
  return { params, steps, errors };
}

export function planLength(waypoints) {
  let len = 0;
  for (let i = 0; i < waypoints.length - 1; i++) len += waypoints[i].pos.distanceTo(waypoints[i + 1].pos);
  return len;
}
