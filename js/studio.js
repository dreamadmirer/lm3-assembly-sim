import * as THREE from 'three';

/**
 * 产品影棚环境（对齐 LEBAI 官方 LM3 渲染图的柔光棚）：
 * - 浅灰墙面房间 + 中央点光源，形成柔和的墙面临近渐变
 * - 顶部巨型柔光板（主光，世界 +Z 方向）
 * - 左右两条长条柔光箱，在缎面白壳的圆柱表面上拉出细长高光
 * - 地板微弱反光板，补足底部暗部
 * 经 PMREMGenerator.fromScene() 采样后用作 scene.environment。
 */
export function makeStudioEnvScene() {
  const scene = new THREE.Scene();

  // 墙面：BackSide 盒子，由点光源照亮产生渐变
  const roomGeo = new THREE.BoxGeometry();
  roomGeo.deleteAttribute('uv');
  const room = new THREE.Mesh(
    roomGeo,
    new THREE.MeshStandardMaterial({ side: THREE.BackSide, color: 0x9aa0aa, roughness: 1, metalness: 0 })
  );
  room.scale.set(24, 24, 24);
  scene.add(room);

  const lamp = new THREE.PointLight(0xffffff, 220, 60, 2);
  lamp.position.set(0, 0, 5);
  scene.add(lamp);

  // 柔光箱：双面自发光平面，朝向原点
  const softbox = (intensity, w, h, x, y, z, hex = 0xffffff) => {
    const mat = new THREE.MeshBasicMaterial({ side: THREE.DoubleSide });
    mat.color.setHex(hex).multiplyScalar(intensity);
    const m = new THREE.Mesh(new THREE.PlaneGeometry(w, h), mat);
    m.position.set(x, y, z);
    m.lookAt(0, 0, 0);
    scene.add(m);
  };

  // 顶部大面积柔光（世界 +Z = 上方）
  softbox(3.6, 11, 11, 0, 0, 11);
  // 左前长条柔光箱（略暖，主高光条）
  softbox(16, 1.6, 9, -9.5, 5.5, 4, 0xfff4e6);
  // 右侧长条柔光箱（冷色，副高光条）
  softbox(7, 1.4, 9, 9.5, -3.5, 4, 0xdcebff);
  // 背景弱光（轮廓过渡）
  softbox(2.5, 7, 3, 0, -10.5, 5);
  // 地板反光板（提升底面亮度）
  softbox(0.9, 11, 11, 0, 0, -11, 0xe8ecef);

  scene.dispose = () => {
    const resources = new Set();
    scene.traverse((o) => {
      if (o.isMesh) { resources.add(o.geometry); resources.add(o.material); }
    });
    resources.forEach((r) => r.dispose());
  };
  return scene;
}
