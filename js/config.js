/* 场景与实验配置：单位 m，世界坐标为 Z 轴向上（与 CAD 模型一致） */

export const ROBOT_BASE = [0, -0.28, 0];
export const ROBOT_YAW = Math.PI / 4; // 让机器人正面朝向 +Y

export const TABLE = {
  width: 1.22,
  depth: 0.94,
  thickness: 0.045,
  topZ: 0,
  center: [0, -0.05],
};

export const BINS = {
  A: { id: 'A', name: 'A 盒', pos: [-0.41, -0.02], size: [0.26, 0.19, 0.1], color: 0x2c7a7b, tint: '#2c7a7b' },
  B: { id: 'B', name: 'B 盒', pos: [0.41, -0.02], size: [0.26, 0.19, 0.1], color: 0x9c4221, tint: '#9c4221' },
};

/* 零件：category 用于自动分拣（round=回转体 / poly=多面体） */
export const OBJECTS = [
  { id: 'cube', name: '立方块', type: 'cube', size: [0.07, 0.07, 0.07], pos: [-0.21, 0.12], yaw: 0.14, color: 0x3d7bfd, category: 'poly', mass: 0.11 },
  { id: 'cyl', name: '圆柱销', type: 'cylinder', radius: 0.03, height: 0.09, pos: [0.01, 0.09], yaw: 0, color: 0xf5a524, category: 'round', mass: 0.09 },
  { id: 'hex', name: '六棱柱', type: 'hex', radius: 0.04, height: 0.075, pos: [0.22, 0.12], yaw: 0.44, color: 0x1fae6b, category: 'poly', mass: 0.10 },
  { id: 'tri', name: '三棱柱', type: 'tri', radius: 0.045, height: 0.075, pos: [-0.17, 0.29], yaw: -0.32, color: 0xf05c5c, category: 'poly', mass: 0.09 },
  { id: 'ball', name: '球体', type: 'sphere', radius: 0.034, pos: [0.03, 0.29], yaw: 0, color: 0x8b5cf6, category: 'round', mass: 0.07 },
  { id: 'lblock', name: 'L 形块', type: 'lblock', size: [0.08, 0.08, 0.07], pos: [0.23, 0.29], yaw: 0.55, color: 0x0aa2c0, category: 'poly', mass: 0.13 },
];

/* 各关节软限位（LM3 各轴机械上无限位，这里按工程习惯取 ±170°） */
export const JOINT_LIMITS = [
  [-175, 175], [-170, 170], [-170, 170], [-175, 175], [-175, 175], [-175, 175],
].map(([a, b]) => [a * Math.PI / 180, b * Math.PI / 180]);

/* 常用姿态（度） */
export const POSE_HOME = [0, 30, 60, -30, -90, 0];
export const POSE_READY = [-17, 10, 93, -13, -90, -17];
export const ROBOT_POS = [0, -0.28, 0];
