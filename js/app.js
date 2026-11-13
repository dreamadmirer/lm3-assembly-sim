import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { Robot } from './robot.js';
import { buildWorld } from './scene.js';
import { BINS, OBJECTS, POSE_READY } from './config.js';
import { PathViz } from './pathviz.js';
import { planTask, planLength, programFor, parseProgram, DEFAULT_PARAMS, STEP_COLORS } from './planner.js';

window.__errors = [];
addEventListener('error', (e) => window.__errors.push('ERR ' + (e.message || e.type)));
addEventListener('unhandledrejection', (e) => window.__errors.push('REJ ' + String((e.reason && (e.reason.stack || e.reason.message)) || e.reason)));

const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;
const $ = (id) => document.getElementById(id);
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const easeInOut = (t) => (t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2);
const nextFrame = () => new Promise((r) => requestAnimationFrame(r));

class App {
  constructor() {
    this.scene = new THREE.Scene();
    this.clock = new THREE.Clock();
    this.busy = false;
    this.sorting = false;
    this.mode = 'pick';
    this.selected = null;
    this.targetBin = 'A';
    this.stats = { done: 0, attempts: 0, lastCycle: null, slots: { A: 0, B: 0 } };
    this.teachPoints = [];
    this.viz = null;
    this.trailOn = true;
    this.codeDirty = false;
    this.params = { ...DEFAULT_PARAMS };
  }

  async init() {
    const canvas = $('view');
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.setSize(innerWidth, innerHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.0;

    this.camera = new THREE.PerspectiveCamera(36, innerWidth / innerHeight, 0.05, 60);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(1.16, 2.12, 1.06);

    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.target.set(0.0, 0.02, 0.27);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.075;
    this.controls.minDistance = 0.7;
    this.controls.maxDistance = 4.2;
    this.controls.maxPolarAngle = Math.PI * 0.495;
    this.controls.update();

    const pmrem = new THREE.PMREMGenerator(this.renderer);
    this.scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    this.scene.environmentIntensity = 0.42;

    this.world = buildWorld(this.scene);
    this.viz = new PathViz(this.scene);

    this.robot = new Robot(this.scene);
    await this.robot.load('./assets/lm3.glb', (p) => {
      const t = $('loaderText');
      if (t) t.textContent = `解析 LM3 模型 ${Math.round(p * 100)}%`;
    });
    this.robot.setJoints(POSE_READY.map((d) => d * D2R));
    this.robot.setGripper(0.075, true);

    this.raycaster = new THREE.Raycaster();
    this.pointer = new THREE.Vector2();

    this.buildUI();
    this.bindEvents();
    this.buildCodePanel();
    this.selectBin('A');
    this.log('仿真环境已加载 · LM3 模型解析完成（7 连杆 / 6 关节）');
    this.log('就绪：选择零件后执行抓取，或切换到自动分拣');

    addEventListener('resize', () => this.onResize());
    this.loop();

    const loader = $('loader');
    if (loader) {
      requestAnimationFrame(() => {
        loader.classList.add('hide');
        setTimeout(() => loader.remove(), 600);
      });
    }

    window.__app = this;
    const view = new URLSearchParams(location.search).get('view');
    if (view) setTimeout(() => this.setView(view), 300);
    if (new URLSearchParams(location.search).has('auto')) {
      setTimeout(() => this.sortAll(), 500);
    }
    if (new URLSearchParams(location.search).has('selftest2')) {
      setTimeout(async () => {
        const obj = this.world.objects[3];
        const t0 = performance.now();
        const ok = await this.pickAndPlace(obj, 'A', { params: { ...this.params, tJoint: 0.08, tLinear: 0.08 } });
        document.title = `RUN ${ok ? 'OK' : 'FAIL'} ${((performance.now() - t0) / 1000).toFixed(1)}s done=${this.stats.done} trail=${this.viz.trailLength().toFixed(2)} err=${(window.__errors || []).join('|')}`;
      }, 1200);
    }
    if (new URLSearchParams(location.search).has('selftest')) {
      setTimeout(() => {
        const obj = this.world.objects[0];
        const plan = planTask({
          obj, bin: BINS[this.targetBin], slot: this.nextSlot(this.targetBin), params: this.params, robot: this.robot,
        });
        if (plan.error) {
          document.title = `SELFTEST FAIL ${plan.error}`;
          return;
        }
        plan.waypoints.forEach((pt, i) => this.viz.addWaypoint(pt, i));
        this.viz.drawPlanLines(plan.waypoints);
        document.title = `SELFTEST OK ${plan.waypoints.length}pts ${plan.steps.length}steps L=${planLength(plan.waypoints).toFixed(2)}`;
      }, 900);
    }
  }

  /* ------------------------------------------------------------ UI */
  buildUI() {
    // 零件列表
    const list = $('objList');
    this.objItems = new Map();
    this.world.objects.forEach((obj, i) => {
      const cfg = obj.userData.cfg;
      const li = document.createElement('li');
      li.innerHTML = `<span class="dotc" style="background:#${cfg.color.toString(16).padStart(6, '0')}"></span>
        <span class="nm">${cfg.name}</span><span class="meta">P${i + 1}</span>`;
      li.onclick = () => this.selectObject(obj);
      list.appendChild(li);
      this.objItems.set(obj, li);
    });

    // 关节栏
    const joints = $('joints');
    this.jointEls = [];
    for (let i = 0; i < 6; i++) {
      const row = document.createElement('div');
      row.className = 'jrow';
      row.innerHTML = `<span class="jn">J${i + 1}</span><div class="track"><i class="fill"></i></div><b>0°</b>`;
      joints.appendChild(row);
      this.jointEls.push({ fill: row.querySelector('.fill'), val: row.querySelector('b') });
    }

    // 示教滑块
    const sliders = $('sliders');
    this.sliderEls = [];
    for (let i = 0; i < 6; i++) {
      const row = document.createElement('div');
      row.className = 'slider-row';
      row.innerHTML = `<label>J${i + 1}</label><input type="range" min="-170" max="170" step="0.5" value="0"><output>0°</output>`;
      sliders.appendChild(row);
      const input = row.querySelector('input');
      const out = row.querySelector('output');
      input.addEventListener('input', () => {
        if (this.busy) return;
        const q = this.robot.q.slice();
        q[i] = parseFloat(input.value) * D2R;
        this.robot.setJoints(q);
        out.textContent = `${parseFloat(input.value).toFixed(0)}°`;
      });
      this.sliderEls.push({ input, out });
    }
  }

  bindEvents() {
    // 模式
    document.querySelectorAll('#modes .mode').forEach((btn) => {
      btn.onclick = () => {
        if (this.busy) return;
        document.querySelectorAll('#modes .mode').forEach((b) => b.classList.remove('active'));
        btn.classList.add('active');
        this.mode = btn.dataset.mode;
        $('pickControls').classList.toggle('hidden', this.mode === 'teach');
        $('teachControls').classList.toggle('hidden', this.mode !== 'teach');
        if (this.mode === 'teach') this.syncSliders();
        this.log(`切换到「${btn.textContent}」模式`);
      };
    });

    // 目标料盒
    document.querySelectorAll('#binSeg button').forEach((btn) => {
      btn.onclick = () => this.selectBin(btn.dataset.bin);
    });

    $('btnPick').onclick = () => {
      if (this.selected) this.pickAndPlace(this.selected, this.targetBin);
    };
    $('btnSortAll').onclick = () => {
      if (this.sorting) {
        this.sorting = false;
        $('btnSortAll').textContent = '一键全部分拣';
        this.log('已请求停止自动分拣');
      } else {
        this.sortAll();
      }
    };
    $('btnReset').onclick = () => this.reset();

    $('btnZero').onclick = () => this.goJoints([0, 0, 0, 0, 0, 0], 1.6, '已回到零位');
    $('btnReady').onclick = () => this.goJoints(POSE_READY.map((d) => d * D2R), 1.4, '已回到工作位');
    $('btnRecord').onclick = () => {
      this.teachPoints.push(this.robot.q.slice());
      this.renderTeachList();
      this.log(`记录示教点 #${this.teachPoints.length}`);
    };
    $('btnPlay').onclick = () => this.playTeach();

    // 视角
    document.querySelectorAll('.viewbar button[data-view]').forEach((btn) => {
      btn.onclick = () => {
        document.querySelectorAll('.viewbar button[data-view]').forEach((b) => b.classList.remove('active'));
        btn.classList.add('active');
        this.setView(btn.dataset.view);
      };
    });

    $('btnTrail').onclick = () => {
      this.trailOn = !this.trailOn;
      this.viz.setTrailVisible(this.trailOn);
      $('btnTrail').classList.toggle('on', this.trailOn);
      this.log(this.trailOn ? '轨迹显示：开启' : '轨迹显示：关闭');
    };

    // 画布点选
    const canvas = $('view');
    let down = null;
    canvas.addEventListener('pointerdown', (e) => { down = { x: e.clientX, y: e.clientY }; });
    canvas.addEventListener('pointermove', (e) => {
      if (this.busy) { canvas.style.cursor = 'default'; return; }
      const rect = canvas.getBoundingClientRect();
      this.pointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
      this.pointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
      this.raycaster.setFromCamera(this.pointer, this.camera);
      const hit = this.raycaster.intersectObjects(this.world.objects, true).length ||
        this.raycaster.intersectObjects(Object.values(this.world.bins), true).length;
      canvas.style.cursor = hit ? 'pointer' : 'default';
    });
    canvas.addEventListener('pointerup', (e) => {
      if (!down) return;
      const moved = Math.hypot(e.clientX - down.x, e.clientY - down.y);
      down = null;
      if (moved > 5 || this.busy) return;
      this.onPick(e);
    });
  }

  onResize() {
    this.camera.aspect = innerWidth / innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(innerWidth, innerHeight);
  }

  onPick(e) {
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.pointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    this.pointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    this.raycaster.setFromCamera(this.pointer, this.camera);

    const objHits = this.raycaster.intersectObjects(this.world.objects, true);
    if (objHits.length) {
      let node = objHits[0].object;
      while (node.parent && !this.world.objects.includes(node)) node = node.parent;
      this.selectObject(node);
      return;
    }
    const binHits = this.raycaster.intersectObjects(Object.values(this.world.bins), true);
    if (binHits.length) {
      let node = binHits[0].object;
      while (node.parent && !Object.values(this.world.bins).includes(node)) node = node.parent;
      const id = node.userData.id;
      this.selectBin(id);
      if (this.selected && !this.busy) this.log(`目标料盒切换为 ${id} 盒`);
    }
  }

  selectObject(obj) {
    if (this.busy) return;
    if (obj && obj.userData.inBin) {
      this.toast(`${obj.userData.cfg.name} 已在 ${obj.userData.inBin} 盒中`);
      return;
    }
    if (this.selected === obj) {
      // 取消选择
      obj.userData.ring.material.opacity = 0;
      this.selected = null;
      $('btnPick').disabled = true;
      this.objItems.forEach((li) => li.classList.remove('active'));
      return;
    }
    if (this.selected) this.selected.userData.ring.material.opacity = 0;
    this.selected = obj;
    obj.userData.ring.material.opacity = 0.95;
    this.objItems.forEach((li, o) => li.classList.toggle('active', o === obj));
    $('btnPick').disabled = false;
    const cfg = obj.userData.cfg;
    this.log(`选中零件：${cfg.name}（${cfg.category === 'round' ? '回转体' : '多面体'}）`);
    this.refreshProgram(true);
  }

  selectBin(id) {
    this.targetBin = id;
    document.querySelectorAll('#binSeg button').forEach((b) => b.classList.toggle('active', b.dataset.bin === id));
    Object.entries(this.world.bins).forEach(([k, bin]) => {
      const on = k === id;
      bin.children.forEach((c) => { if (c.isSprite) c.material.opacity = on ? 1 : 0.6; });
    });
    const ring = this.world.targetRing;
    const bin = this.world.bins[id];
    if (ring && bin) {
      ring.position.set(bin.position.x, bin.position.y, 0.0018);
      ring.visible = true;
    }
    if (this.codeArea) this.refreshProgram(true);
  }

  syncSliders() {
    for (let i = 0; i < 6; i++) {
      const deg = this.robot.q[i] * R2D;
      this.sliderEls[i].input.value = deg.toFixed(1);
      this.sliderEls[i].out.textContent = `${deg.toFixed(0)}°`;
    }
  }

  renderTeachList() {
    const el = $('teachList');
    el.innerHTML = '';
    this.teachPoints.slice(-4).forEach((q, i) => {
      const d = document.createElement('div');
      d.innerHTML = `<span>示教点 ${this.teachPoints.length - Math.min(4, this.teachPoints.length) + i + 1}</span>
        <span>${q.map((v) => (v * R2D).toFixed(0)).join(' / ')}</span>`;
      el.appendChild(d);
    });
  }

  async playTeach() {
    if (this.busy || !this.teachPoints.length) {
      if (!this.teachPoints.length) this.toast('还没有记录示教点');
      return;
    }
    this.setBusy(true, '回放示教轨迹');
    for (const q of this.teachPoints) await this.robot.moveJ(q, 1.4);
    this.log(`回放完成：${this.teachPoints.length} 个示教点`);
    this.setBusy(false, '就绪');
    this.syncSliders();
  }

  /* ------------------------------------------------------------ 运动与抓取 */
  async goJoints(q, dur, msg) {
    if (this.busy) return;
    this.setBusy(true, '运动中');
    await this.robot.moveJ(q, dur);
    this.setBusy(false, '就绪');
    this.syncSliders();
    if (msg) this.log(msg);
  }

  async approach(pos, yaw, dur) {
    // 先关节空间到上方，再直线下降
    const sol = this.robot.solveTarget(pos, yaw);
    if (!sol) return false;
    await this.robot.moveJ(sol.q, dur);
    await this.robot.moveL(pos, yaw, 0, 0.45);
    return true;
  }

  async pickAndPlace(obj, binId, opts = {}) {
    if (this.busy) return false;
    const t0 = performance.now();
    const cfg = obj.userData.cfg;

    if (this.selected) this.selected.userData.ring.material.opacity = 0;
    this.selected = null;
    this.objItems.forEach((li) => li.classList.remove('active'));
    $('btnPick').disabled = true;

    const params = { ...DEFAULT_PARAMS, ...(opts.params || {}) };
    this.params = params;
    const slot = this.nextSlot(binId);
    const plan = planTask({
      obj, bin: BINS[binId], slot, params, robot: this.robot, steps: opts.steps,
    });
    if (plan.error) return this.fail(plan.error);

    this.setBusy(true, '路径规划');
    await this.animatePlanning(plan);

    this.setBusy(true, `搬运 ${cfg.name}`);
    this.robot.setGripper(Math.min(0.094, plan.width + params.openGap));
    await sleep(160);

    this.viz.beginTrail();
    this.viz.setTrailVisible(this.trailOn);
    let attached = false;
    let failed = null;

    try {
      for (const step of plan.steps) {
        if (step.target) {
          const wi = plan.waypoints.findIndex((w) => w.key === step.target);
          this.viz.setPhaseColor(STEP_COLORS[step.target] || 0x0f9d8f);
          this.viz.focusSegment(wi - 1);
        } else if (step.cmd === 'gripper') {
          this.viz.setPhaseColor(step.arg === 'close' ? 0x14b8a6 : 0x8b5cf6);
        } else {
          this.viz.setPhaseColor(0x64748b);
        }

        if (step.cmd === 'move_j') {
          const tg = plan.targets[step.target];
          if (!tg) { failed = `缺少路径点 ${step.target}`; break; }
          await this.robot.moveJ(tg.q, params.tJoint);
        } else if (step.cmd === 'move_l') {
          const tg = plan.targets[step.target];
          if (!tg) { failed = `缺少路径点 ${step.target}`; break; }
          await this.robot.moveL(tg.pos, tg.yaw, 0, params.tLinear);
        } else if (step.cmd === 'gripper') {
          if (step.arg === 'close') {
            this.robot.setGripper(Math.max(0.016, plan.width - 0.002));
            await sleep(420);
            if (!attached) {
              this.robot.attach(obj);
              obj.userData.busy = true;
              attached = true;
              this.log(`夹爪闭合 ${(this.robot.gripperOpen * 1000).toFixed(0)} mm · 抓取 ${cfg.name}`);
            }
          } else {
            this.robot.setGripper(0.092);
            await sleep(240);
            if (attached) {
              await this.releaseAndDrop(obj, plan);
              attached = false;
            }
          }
        } else if (step.cmd === 'home') {
          this.viz.fadePlan(0.3);
          await this.robot.moveJ(POSE_READY.map((d) => d * D2R), params.tJoint);
        } else if (step.cmd === 'wait') {
          await sleep(step.seconds * 1000);
        }
      }
    } finally {
      this.viz.endTrail();
    }

    if (failed) return this.fail(failed);
    if (attached) this.dropCarried();

    this.viz.fadePlan(0.45);
    const cycle = (performance.now() - t0) / 1000;
    this.stats.done += 1;
    this.stats.attempts += 1;
    this.stats.lastCycle = cycle;
    this.updateObjectList();
    this.updateStats();

    this.setBusy(false, '就绪');
    this.log(`运行轨迹 ${this.viz.trailLength().toFixed(2)} m · 节拍 ${cycle.toFixed(1)} s`);
    this.toast(`${cfg.name} 已放入 ${binId} 盒 · 节拍 ${cycle.toFixed(1)} s`);
    this.syncSliders();
    return true;
  }

  /* 规划过程可视化：路径点逐个生成 + 连线 */
  async animatePlanning(plan) {
    const pts = plan.waypoints;
    this.viz.clearPlan();
    this.viz.clearTrail();
    for (let i = 0; i < pts.length; i++) {
      this.viz.addWaypoint(pts[i], i);
      this.setState(`路径规划 ${i + 1}/${pts.length}`);
      await sleep(110);
    }
    this.viz.drawPlanLines(pts);
    this.log(`路径规划完成：${pts.length} 个路径点 · 空间距离 ${planLength(pts).toFixed(2)} m`);
  }

  async releaseAndDrop(obj, plan) {
    const p = new THREE.Vector3().setFromMatrixPosition(this.robot.tcp());
    this.robot.releaseTo(this.scene);
    obj.userData.busy = false;
    obj.userData.inBin = plan.binId;
    this.stats.slots[plan.binId] = (this.stats.slots[plan.binId] || 0) + 1;
    await this.dropTo(obj, new THREE.Vector3(p.x, p.y, plan.h * 0.5 + 0.009), 0.3);
    this.log(`放置完成：${plan.cfg.name} → ${plan.binId} 盒`, 'ok');
  }

  /* ------------------------------------------------------------ 代码面板 */
  buildCodePanel() {
    this.codeArea = $('codeArea');
    this.codeHl = $('codeHl');
    if (!this.codeArea) return;

    this.codeArea.addEventListener('input', () => {
      this.codeDirty = true;
      this.highlightCode();
      this.showCodeMsg('已修改，点击「运行」执行', 'dirty');
    });
    this.codeArea.addEventListener('scroll', () => {
      this.codeHl.style.transform = `translate(${-this.codeArea.scrollLeft}px, ${-this.codeArea.scrollTop}px)`;
    });
    this.codeArea.addEventListener('keydown', (e) => {
      if (e.key !== 'Tab') return;
      e.preventDefault();
      const el = this.codeArea;
      const a = el.selectionStart, b = el.selectionEnd;
      el.value = el.value.slice(0, a) + '  ' + el.value.slice(b);
      el.selectionStart = el.selectionEnd = a + 2;
      this.codeDirty = true;
      this.highlightCode();
    });

    $('btnRunCode').onclick = () => this.runCode();
    $('btnResetCode').onclick = () => {
      this.refreshProgram(true);
      this.showCodeMsg('已还原为当前零件程序', '');
    };
    $('btnFold').onclick = () => {
      const panel = $('codePanel');
      panel.classList.toggle('folded');
      $('btnFold').textContent = panel.classList.contains('folded') ? '▴' : '▾';
    };
    this.refreshProgram(true);
  }

  refreshProgram(force = false) {
    if (!this.codeArea) return;
    if (this.codeDirty && !force) return;
    const obj = this.selected || this.world.objects.find((o) => !o.userData.inBin);
    const label = $('codeTarget');
    if (!obj) {
      this.codeArea.value = '# 所有零件已进入料盒\n# 点击「重置实验场景」重新开始';
      label.textContent = '无待抓取零件';
      $('btnRunCode').disabled = true;
      this.codeDirty = false;
      this.highlightCode();
      return;
    }
    $('btnRunCode').disabled = false;
    label.textContent = `${obj.userData.cfg.name} → ${this.targetBin} 盒`;
    this.codeArea.value = programFor({ cfg: obj.userData.cfg, binId: this.targetBin, params: this.params });
    this.codeDirty = false;
    this.highlightCode();
  }

  highlightCode() {
    if (!this.codeHl) return;
    const esc = (t) => t.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
    const html = this.codeArea.value.split('\n').map((line) => {
      const hash = line.indexOf('#');
      const code = hash >= 0 ? line.slice(0, hash) : line;
      const cmt = hash >= 0 ? line.slice(hash) : '';
      let out;
      const assign = code.match(/^(\s*)([A-Z][A-Z0-9_]*)(\s*=\s*)(-?\d+(?:\.\d+)?)(\s*)$/);
      const call = code.match(/^(\s*)(move_j|move_l|gripper|home|wait)(\s+)?(\w+)?(\s*)$/);
      if (assign) {
        out = `${assign[1]}<span class="tok-key">${assign[2]}</span>${assign[3]}<span class="tok-num">${assign[4]}</span>${assign[5]}`;
      } else if (call) {
        out = `${call[1]}<span class="tok-cmd">${call[2]}</span>${call[3] || ''}${call[4] ? `<span class="tok-arg">${call[4]}</span>` : ''}${call[5] || ''}`;
      } else {
        out = esc(code);
      }
      return out + (cmt ? `<span class="tok-cmt">${esc(cmt)}</span>` : '');
    }).join('\n');
    this.codeHl.innerHTML = html + '\n';
  }

  showCodeMsg(text, kind = '') {
    const el = $('codeMsg');
    if (!el) return;
    el.textContent = text;
    el.className = kind;
  }

  async runCode() {
    if (this.busy) return;
    const obj = this.selected;
    if (!obj) {
      this.showCodeMsg('请先在左侧列表或 3D 视图中选择零件', 'error');
      this.toast('请先选择零件');
      return;
    }
    const { params, steps, errors } = parseProgram(this.codeArea.value);
    if (errors.length) {
      this.showCodeMsg(errors[0], 'error');
      this.toast(errors[0]);
      return;
    }
    this.showCodeMsg('程序已解析，开始规划…', 'ok');
    await this.pickAndPlace(obj, this.targetBin, { params, steps });
    this.showCodeMsg('执行完成，可继续修改参数', 'ok');
  }

  setState(text) {
    $('stateText').textContent = text;
    $('stateDot').parentElement.classList.add('busy');
  }

  nextSlot(binId) {
    const n = this.stats.slots[binId];
    const grid = [
      { dx: -0.06, dy: -0.045, yaw: 0 },
      { dx: 0.06, dy: -0.045, yaw: 0.1 },
      { dx: -0.06, dy: 0.048, yaw: -0.08 },
      { dx: 0.06, dy: 0.048, yaw: 0.06 },
    ];
    return grid[n % 4];
  }

  dropCarried() {
    const holder = this.robot.holder;
    if (!holder || !holder.children.length) return;
    const obj = holder.children[0];
    obj.updateMatrixWorld(true);
    const p = new THREE.Vector3().setFromMatrixPosition(obj.matrixWorld);
    this.robot.releaseTo(this.scene);
    obj.userData.inBin = null;
    obj.userData.busy = false;
    const h = obj.userData.height || 0.05;
    this.dropTo(obj, new THREE.Vector3(p.x, p.y, h * 0.5), 0.35);
    this.log(`异常回收：${obj.userData.cfg.name} 已落回台面`, 'warn');
  }

  async dropTo(obj, target, dur = 0.3) {
    const p0 = obj.position.clone();
    const start = performance.now();
    for (;;) {
      const t = Math.min(1, (performance.now() - start) / (dur * 1000));
      const e = t * t;
      obj.position.set(
        p0.x + (target.x - p0.x) * Math.min(1, t * 1.6),
        p0.y + (target.y - p0.y) * Math.min(1, t * 1.6),
        p0.z + (target.z - p0.z) * e
      );
      if (t >= 1) break;
      await nextFrame();
    }
    obj.position.copy(target);
  }

  fail(msg) {
    this.stats.attempts += 1;
    this.dropCarried();
    this.setBusy(false, '异常');
    this.log(`失败：${msg}`, 'warn');
    this.toast(msg);
    this.updateStats();
    return false;
  }

  async sortAll() {
    if (this.busy) return;
    this.sorting = true;
    $('btnSortAll').textContent = '停止分拣';
    this.log('自动分拣任务开始（回转体 → A 盒，多面体 → B 盒）');
    while (this.sorting) {
      const next = this.world.objects.find((o) => !o.userData.inBin);
      if (!next) break;
      const bin = next.userData.cfg.category === 'round' ? 'A' : 'B';
      this.selectBin(bin);
      const ok = await this.pickAndPlace(next, bin, { auto: true });
      if (!ok) {
        this.sorting = false;
        break;
      }
      await sleep(260);
    }
    this.sorting = false;
    $('btnSortAll').textContent = '一键全部分拣';
    if (!this.busy) {
      const left = this.world.objects.filter((o) => !o.userData.inBin).length;
      this.log(left ? `分拣中止，剩余 ${left} 件` : '自动分拣完成 · 全部零件已入盒', 'ok');
      if (!left) this.toast('自动分拣完成');
    }
  }

  async reset() {
    if (this.busy) return;
    this.sorting = false;
    this.setBusy(true, '重置中');
    this.robot.setGripper(0.09);
    await sleep(250);
    for (const obj of this.world.objects) {
      if (obj.parent !== this.scene) this.scene.attach(obj);
      const cfg = obj.userData.cfg;
      obj.position.set(cfg.pos[0], cfg.pos[1], 0);
      obj.rotation.set(0, 0, cfg.yaw || 0);
      obj.userData.inBin = null;
      obj.userData.busy = false;
      obj.userData.ring.material.opacity = 0;
      obj.updateMatrixWorld(true);
    }
    this.selected = null;
    if (this.viz) { this.viz.clearPlan(); this.viz.clearTrail(); this.viz.endTrail(); }
    this.stats = { done: 0, attempts: 0, lastCycle: null, slots: { A: 0, B: 0 } };
    this.teachPoints = [];
    this.viz = null;
    this.trailOn = true;
    this.codeDirty = false;
    this.params = { ...DEFAULT_PARAMS };
    this.renderTeachList();
    this.updateObjectList();
    this.updateStats();
    $('btnPick').disabled = true;
    this.objItems.forEach((li) => li.classList.remove('active'));
    await this.robot.moveJ(POSE_READY.map((d) => d * D2R), 1.4);
    this.log('实验场景已重置');
    this.setBusy(false, '就绪');
    this.syncSliders();
    this.refreshProgram(true);
  }

  /* ------------------------------------------------------------ 状态与日志 */
  setBusy(v, text) {
    this.busy = v;
    $('stateText').textContent = text || (v ? '运行中' : '就绪');
    $('stateDot').parentElement.classList.toggle('busy', v);
    $('btnPick').disabled = v || !this.selected;
    $('btnSortAll').disabled = v && !this.sorting;
  }

  updateObjectList() {
    const left = this.world.objects.filter((o) => !o.userData.inBin).length;
    $('objCount').textContent = left;
    this.world.objects.forEach((o, i) => {
      const li = this.objItems.get(o);
      li.classList.toggle('done', !!o.userData.inBin);
      li.querySelector('.meta').textContent = o.userData.inBin ? `${o.userData.inBin} 盒` : `P${i + 1}`;
    });
  }

  updateStats() {
    $('statDone').textContent = this.stats.done;
    $('statCycle').textContent = this.stats.lastCycle ? `${this.stats.lastCycle.toFixed(1)}s` : '—';
    const rate = this.stats.attempts ? Math.round((this.stats.done / this.stats.attempts) * 100) : 100;
    $('statRate').textContent = `${rate}%`;
  }

  log(msg, cls = '') {
    const ul = $('log');
    const li = document.createElement('li');
    const d = new Date();
    li.className = cls;
    li.innerHTML = `<time>${String(d.getHours()).padStart(2, '0')}:${String(d.getMinutes()).padStart(2, '0')}:${String(d.getSeconds()).padStart(2, '0')}</time><span>${msg}</span>`;
    ul.prepend(li);
    while (ul.children.length > 5) ul.removeChild(ul.lastChild);
  }

  toast(msg) {
    const t = $('toast');
    t.textContent = msg;
    t.classList.add('show');
    clearTimeout(this._toastT);
    this._toastT = setTimeout(() => t.classList.remove('show'), 2400);
  }

  setView(kind) {
    const views = {
      iso: [[1.16, 2.12, 1.06], [0.0, 0.02, 0.27]],
      front: [[0.03, 2.25, 0.72], [0.0, 0.0, 0.24]],
      top: [[0.0, -0.2, 2.45], [0, -0.1, 0.02]],
    };
    const [p, t] = views[kind] || views.iso;
    this.flyCamera(new THREE.Vector3(...p), new THREE.Vector3(...t));
  }

  async flyCamera(pos, target) {
    const p0 = this.camera.position.clone();
    const t0 = this.controls.target.clone();
    const start = performance.now();
    for (;;) {
      const k = Math.min(1, (performance.now() - start) / 700);
      const e = easeInOut(k);
      this.camera.position.lerpVectors(p0, pos, e);
      this.controls.target.lerpVectors(t0, target, e);
      this.controls.update();
      if (k >= 1) break;
      await nextFrame();
    }
  }

  updateHud() {
    const q = this.robot.q;
    for (let i = 0; i < 6; i++) {
      const deg = q[i] * R2D;
      const el = this.jointEls[i];
      el.val.textContent = `${deg.toFixed(0)}°`;
      const pct = Math.min(50, Math.abs(deg) / 170 * 50);
      el.fill.style.width = `${pct}%`;
      el.fill.style.left = deg >= 0 ? '50%' : `${50 - pct}%`;
      el.fill.style.background = Math.abs(deg) > 150 ? '#f59e0b' : '#0d9488';
    }
    const T = this.robot.tcp();
    const p = new THREE.Vector3().setFromMatrixPosition(T);
    const e = new THREE.Euler().setFromRotationMatrix(T, 'XYZ');
    $('tcpX').textContent = (p.x * 1000).toFixed(0);
    $('tcpY').textContent = (p.y * 1000).toFixed(0);
    $('tcpZ').textContent = (p.z * 1000).toFixed(0);
    $('tcpRx').textContent = (e.x * R2D).toFixed(0);
    $('tcpRy').textContent = (e.y * R2D).toFixed(0);
    $('tcpRz').textContent = (e.z * R2D).toFixed(0);
    $('gripVal').textContent = `${(this.robot.gripperOpen * 1000).toFixed(0)} mm`;
    $('gripBar').style.width = `${Math.min(100, (this.robot.gripperOpen / 0.095) * 100)}%`;
    if (this.mode === 'teach' && !this.busy) {
      for (let i = 0; i < 6; i++) {
        const deg = q[i] * R2D;
        if (document.activeElement !== this.sliderEls[i].input) {
          this.sliderEls[i].input.value = deg.toFixed(1);
          this.sliderEls[i].out.textContent = `${deg.toFixed(0)}°`;
        }
      }
    }
  }

  loop() {
    const dt = Math.min(0.05, this.clock.getDelta());
    this.robot.update(dt);
    if (this.viz) {
      if (this.viz.recording) {
        this.viz.pushTrail(new THREE.Vector3().setFromMatrixPosition(this.robot.tcp()));
      }
      this.viz.animateHalo(performance.now() / 1000);
    }
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    const now = performance.now();
    if (!this._hudT || now - this._hudT > 90) {
      this._hudT = now;
      this.updateHud();
    }
    requestAnimationFrame(() => this.loop());
  }
}

const app = new App();
app.init().catch((err) => {
  console.error(err);
  const loader = document.getElementById('loader');
  if (loader) {
    loader.classList.add('error');
    const t = document.getElementById('loaderText');
    if (t) t.textContent = String(err.message || err);
    return;
  }
  document.body.insertAdjacentHTML('beforeend',
    `<div style="position:fixed;inset:0;display:grid;place-items:center;background:#eef1f6;font:14px system-ui;color:#0f172a">
      <div style="text-align:center"><b>加载失败</b><p style="color:#64748b">${String(err.message || err)}</p>
      <p style="color:#94a3b8">请通过本地 HTTP 服务打开页面（见 README / start.bat）</p></div>
    </div>`);
});
