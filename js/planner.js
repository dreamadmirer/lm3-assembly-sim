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
    base: { x: robot.root.position.x, y: robot.root.position.y, z: robot.root.position.z },
    homeQ: Array.isArray(robot.q) ? robot.q.slice() : null,
    width: graspWidth(cfg),
    params: { ...p },
    targets, waypoints, steps: (steps && steps.length ? steps : DEFAULT_STEPS).map((s) => ({ ...s })),
  };
}

/* ---------------- 程序文本（官方 lebai_sdk Python 风格） ---------------- */
const WP_ORDER = ['approach', 'grasp', 'lift', 'transit', 'place', 'retreat'];

const STEP_NOTE = {
  approach: '预抓取',
  grasp: '下降到抓取点',
  lift: '提升',
  transit: '转运',
  place: '放置点',
  retreat: '退回',
};

const f3 = (v) => Number(v).toFixed(3);
const f4 = (v) => Number(v).toFixed(4);

export const CLAW_MIN = 0.014; // 夹爪最小开口 m
export const CLAW_MAX = 0.095; // 夹爪最大开口 m

/** 夹持宽度 → 官方 set_claw 的 amplitude（0–100，占最大行程的百分比） */
export function amplitudeFor(width) {
  const pct = ((width - CLAW_MIN) / (CLAW_MAX - CLAW_MIN)) * 100;
  return Math.max(0, Math.min(100, Math.round(pct)));
}

/**
 * 生成官方 SDK 风格的可读 Python 任务程序
 * plan 可只带 {cfg, binId, params}（此时没有解算位姿，仅输出骨架）
 */
export function programFor(plan) {
  const p = { ...DEFAULT_PARAMS, ...(plan.params || {}) };
  const name = (plan.cfg && plan.cfg.name) || '零件';
  const bin = plan.binId || 'A';
  const targets = plan.targets || null;
  const width = plan.width != null ? plan.width : (p.gripW || 0.07);
  const steps = (plan.steps && plan.steps.length ? plan.steps : DEFAULT_STEPS);
  const base = plan.base || { x: 0, y: 0, z: 0 };

  if (!targets) {
    return `# -*- coding: utf-8 -*-
"""LM3 抓取放置任务 · ${name} → ${bin} 盒"""

# 当前零件位姿超出工作空间，无法解算路径点
# 可尝试：减小 APPROACH_H / TRANSIT_Z，或先移动零件
raise RuntimeError("target pose unreachable")
`;
  }

  const qRows = [];
  const wpRows = [];
  for (const k of WP_ORDER) {
    const t = targets[k];
    if (!t) continue;
    qRows.push(`  '${k}': (${t.q.map((v) => f4(v)).join(', ')}),`);
    const bx = t.pos.x - base.x;
    const by = t.pos.y - base.y;
    const bz = t.pos.z - base.z;
    wpRows.push(`  '${k}': {'x': ${f3(bx)}, 'y': ${f3(by)}, 'z': ${f3(bz)}, 'rx': 3.1416, 'ry': 0.0, 'rz': ${f3(t.yaw)}},`);
  }

  const homeQ = plan.homeQ || targets.approach.q;
  const ampClose = amplitudeFor(Math.max(0.016, width - 0.002));

  let n = 0;
  const body = steps.map((s) => {
    n += 1;
    if (s.cmd === 'move_j') {
      const line = `        robot.movej(Q['${s.target}'], A_JOINT, V_JOINT, T_JOINT, 0)`;
      return `        # ${n} ${STEP_NOTE[s.target] || ''}\n${line}\n        robot.wait_move()`;
    }
    if (s.cmd === 'move_l') {
      const line = `        robot.movel(WP['${s.target}'], A_LINEAR, V_LINEAR, T_LINEAR, 0)`;
      return `        # ${n} ${STEP_NOTE[s.target] || ''}\n${line}\n        robot.wait_move()`;
    }
    if (s.cmd === 'gripper') {
      const close = s.arg === 'close';
      const call = close
        ? `        robot.set_claw(GRIP_FORCE, GRIP_AMP)   # 夹紧（力度, 开合度%）`
        : `        robot.set_claw(20, 100)                # 张开到最大`;
      return `        # ${n} ${close ? '夹紧零件' : '松开工件'}\n${call}`;
    }
    if (s.cmd === 'home') {
      return `        # ${n} 回工作位\n        robot.movej(HOME_Q, A_JOINT, V_JOINT, T_JOINT, 0)\n        robot.wait_move()`;
    }
    if (s.cmd === 'wait') return `        time.sleep(${s.seconds.toFixed(2)})   # ${n} 等待`;
    return `        # 未知指令`;
  }).join('\n\n');

  return `# -*- coding: utf-8 -*-
"""LM3 抓取放置任务 · ${name} → ${bin} 盒

官方 Python SDK（lebai_sdk）风格；由仿真平台按当前场景解算生成。
修改参数或路径点后点「运行」即可在仿真中复现。
"""
import time
import lebai_sdk

ROBOT_IP    = "192.168.0.100"   # 控制器 IP
APPROACH_H  = ${f3(p.approachH)}   # 预抓取高度 m（零件上方）
GRASP_RATIO = ${f3(p.graspRatio)}   # 抓取高度 = 零件高 × 比例
TRANSIT_Z   = ${f3(p.transitZ)}   # 转运高度 m（基座系）
PLACE_Z     = ${f3(p.placeZ)}   # 放置点距盒底 m
RETREAT_H   = ${f3(p.retreatH)}   # 退回高度 m
T_JOINT     = ${f3(p.tJoint)}   # 关节运动时间 s（t>0 时 A/V 无效）
T_LINEAR    = ${f3(p.tLinear)}   # 直线运动时间 s
GRIP_W      = ${f3(width)}   # 夹持宽度 m

A_JOINT     = 0.500   # 关节加速度 rad/s2（T_JOINT=0 时生效）
V_JOINT     = 0.450   # 关节速度 rad/s
A_LINEAR    = 0.300   # 空间加速度 m/s2
V_LINEAR    = 0.120   # 空间速度 m/s
GRIP_FORCE  = 60      # 夹紧力 0–100
GRIP_AMP    = ${String(ampClose).padStart(3)}      # 开合度 0–100（% 最大行程）

# 关节角（基座坐标系，单位 rad）
Q = {
${qRows.join('\n')}
}

# 末端位姿（基座坐标系，单位 m / rad；rx=3.1416 表示工具 z 轴垂直向下）
WP = {
${wpRows.join('\n')}
}

# 工作位关节角（rad）
HOME_Q = (${homeQ.map((v) => f4(v)).join(', ')})


def main():
    lebai_sdk.init()
    robot = lebai_sdk.connect(ROBOT_IP, False)   # 建立连接
    robot.stop_sys()                             # 先停臂，确认初始安全状态
    print('state:', robot.get_robot_state())     # IDLE / MOVING / ESTOP
    robot.start_sys()                            # 启动手臂
    print('tcp:', robot.get_kin_data()['actual_tcp_pose'])

    try:
        robot.set_claw(20, 100)                # 0 张开夹爪，准备接近
${body}
    except Exception as e:
        print('任务异常：', e)      # 运动不可达 / 关节超限 / 急停
    finally:
        robot.stop_sys()           # 无论成败都停止手臂


if __name__ == '__main__':
    main()
`;
}

/* ---------------- 程序解析（官方 SDK 语法子集） ---------------- */
const KNOWN_KEYS = Object.keys(LABELS);

const NOOP_METHODS = new Set([
  'init', 'connect', 'disconnect', 'start_sys', 'stop_sys', 'print', 'main',
  'kinematics_forward', 'kinematics_inverse', 'pose_trans', 'pose_add',
  'get_robot_state', 'get_kin_data', 'get_claw', 'set_payload', 'teach_mode',
  'end_teach_mode', 'pause_move', 'resume_move', 'reset',
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

/** 从文本中提取 `名称 = { ... }` 块的内容 */
function extractBlock(text, name) {
  const re = new RegExp(`^${name}\\s*=\\s*\\{`, 'm');
  const m = re.exec(text);
  if (!m) return null;
  const start = m.index + m[0].length;
  const end = text.indexOf('\n}', start);
  return text.slice(start, end < 0 ? text.length : end);
}

/** 解析 Q = {...} 关节角字典（rad） */
function parseQBlock(text) {
  const body = extractBlock(text, 'Q');
  if (!body) return null;
  const out = {};
  for (const line of body.split(/\r?\n/)) {
    const m = line.match(/^\s*['"]?([A-Za-z_]\w*)['"]?\s*:\s*\(([^)]*)\)/);
    if (!m) continue;
    const nums = m[2].split(',').map((x) => parseFloat(x)).filter((v) => Number.isFinite(v));
    if (nums.length === 6) out[m[1].toLowerCase()] = nums;
  }
  return Object.keys(out).length ? out : null;
}

/** 解析 WP = {...} 笛卡尔位姿字典（m / rad） */
function parseWPBlock(text) {
  const body = extractBlock(text, 'WP');
  if (!body) return null;
  const out = {};
  for (const line of body.split(/\r?\n/)) {
    const m = line.match(/^\s*['"]?([A-Za-z_]\w*)['"]?\s*:\s*\{([^}]*)\}/);
    if (!m) continue;
    const pose = {};
    const kv = m[2].matchAll(/['"]?([A-Za-z]+)['"]?\s*:\s*(-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)/g);
    for (const item of kv) pose[item[1]] = parseFloat(item[2]);
    if (['x', 'y', 'z', 'rx', 'ry', 'rz'].every((k) => k in pose)) out[m[1].toLowerCase()] = pose;
  }
  return Object.keys(out).length ? out : null;
}

/** 解析 HOME_Q = (...) */
function parseHome(text) {
  const m = text.match(/^HOME_Q\s*=\s*\(([^)]*)\)/m);
  if (!m) return null;
  const nums = m[1].split(',').map((x) => parseFloat(x)).filter((v) => Number.isFinite(v));
  return nums.length === 6 ? nums : null;
}

/** 解析实参：数字 / 常量名 */
function resolveArgs(args, consts) {
  return args.split(',').map((raw) => {
    const v = raw.trim();
    if (!v) return null;
    if (/^-?\d+(?:\.\d+)?$/.test(v)) return parseFloat(v);
    if (consts[v] != null) return consts[v];
    return null;
  }).filter((v) => v !== null);
}

export function parseProgram(text) {
  const params = { ...DEFAULT_PARAMS };
  const steps = [];
  const errors = [];
  const consts = {};
  const lines = String(text).split(/\r?\n/);
  let depth = 0;
  let doc = false;

  // 先解析位姿字典与常量（与语句顺序无关）
  const Q = parseQBlock(text);
  const WP = parseWPBlock(text);
  const HOME = parseHome(text);

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
      if (/^(for|while)\b/.test(line)) errors.push(`第 ${no} 行：仿真不支持循环，请展开为顺序指令`);
      return;
    }

    // 常量赋值
    const assign = line.match(/^([A-Za-z_]\w*)\s*=\s*(.+)$/);
    if (assign) {
      const name = assign[1];
      const value = assign[2].trim();
      if (/^-?\d+(?:\.\d+)?$/.test(value)) {
        const v = parseFloat(value);
        consts[name] = v;
        const meta = PARAM_META[name.toUpperCase()];
        if (meta) {
          if (v < meta.min || v > meta.max) {
            errors.push(`第 ${no} 行：${name} 应在 ${meta.min} ~ ${meta.max} 之间`);
            return;
          }
          params[meta.key] = v;
        }
      }
      return;
    }

    // 兼容简写：move_j approach / gripper close / await 等
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

    // 函数调用
    const call = line.match(/^(?:[A-Za-z_]\w*\s*\.\s*)*([A-Za-z_]\w*)\s*\((.*)\)\s*:?$/);
    if (!call) {
      errors.push(`第 ${no} 行：无法识别的语句「${line}」`);
      return;
    }
    const meth = call[1].toLowerCase();
    const argsRaw = call[2].trim();
    const nums = resolveArgs(argsRaw, consts);

    if (['movej', 'move_j', 'movel', 'move_l'].includes(meth)) {
      const cmd = meth.startsWith('movej') || meth === 'move_j' ? 'move_j' : 'move_l';
      if (/^HOME(_Q)?\b/i.test(argsRaw)) { steps.push({ cmd: 'home' }); return; }
      const k = waypointKey(argsRaw);
      if (!k) { errors.push(`第 ${no} 行：${meth}() 未指定路径点（应使用 Q['…'] 或 WP['…']）`); return; }
      if (!KNOWN_KEYS.includes(k)) { errors.push(`第 ${no} 行：未知路径点 ${k}`); return; }
      const dict = cmd === 'move_j' ? Q : WP;
      if (!dict || !dict[k]) errors.push(`第 ${no} 行：${cmd === 'move_j' ? 'Q' : 'WP'} 中未定义路径点 '${k}'`);
      const step = { cmd, target: k };
      if (nums.length >= 3) {
        step.a = nums[0]; step.v = nums[1]; step.t = nums[2]; step.r = nums[3] || 0;
        if (nums[2] > 0) step.time = nums[2];
      } else if (nums.length === 1) {
        step.time = nums[0]; // 旧式单参数：直接给时长
      }
      steps.push(step);
      return;
    }

    if (meth === 'wait_move') { steps.push({ cmd: 'wait_move' }); return; }

    if (meth === 'set_claw') {
      const [force, amp] = nums;
      steps.push({ cmd: 'claw', force: force != null ? force : 50, amp: amp != null ? amp : 100 });
      return;
    }

    if (meth.includes('gripper')) {
      if (/open/.test(meth) || /\bopen\b/.test(argsRaw)) steps.push({ cmd: 'gripper', arg: 'open' });
      else if (/close/.test(meth) || /\bclose\b/.test(argsRaw)) steps.push({ cmd: 'gripper', arg: 'close' });
      else errors.push(`第 ${no} 行：gripper_*() 需为 open 或 close`);
      return;
    }

    if (meth === 'home' || meth === 'move_home') { steps.push({ cmd: 'home' }); return; }

    if (['sleep', 'wait', 'delay'].includes(meth)) {
      const s = nums[0];
      if (!Number.isFinite(s) || s < 0 || s > 5) { errors.push(`第 ${no} 行：等待时间无效`); return; }
      steps.push({ cmd: 'wait', seconds: s });
      return;
    }

    if (NOOP_METHODS.has(meth)) return;

    errors.push(`第 ${no} 行：未知指令 ${call[1]}()`);
  });

  if (!steps.length) errors.push('程序中没有可执行的指令');
  return { params, steps, poses: { Q, WP, HOME }, errors };
}

export function planLength(waypoints) {
  let len = 0;
  for (let i = 0; i < waypoints.length - 1; i++) len += waypoints[i].pos.distanceTo(waypoints[i + 1].pos);
  return len;
}
