# 与乐白官方 Python SDK 的对齐说明

对照文档：
- Python 例程 <https://help.lebai.ltd/program/python.html#sdk>
- 运动接口 <https://help.lebai.ltd/sdk/motion.html>

仿真平台中的「任务程序」面板不再使用自造的 `LebaiRobot` 类，而是**按官方 `lebai_sdk` 的调用序列与参数语义执行**。实现见 `js/lebai_sim.js`（仿真 SDK 层）、`js/planner.js`（程序生成/解析）、`js/app.js`（执行器与安全流程）。

## 1. 差异对照（改造前 → 官方规范 → 现状）

| 环节 | 改造前实现 | 官方 SDK 规范 | 现在 |
| --- | --- | --- | --- |
| 引入/初始化 | `from lebai import LebaiRobot` | `import lebai_sdk; lebai_sdk.init()` | ✅ `lebai_sdk.init()` |
| 建立连接 | `robot = LebaiRobot(ip)`、`robot.login()` | `lebai = lebai_sdk.connect(ip, False)` | ✅ `connect(ip)` |
| 使能/停止 | `start_task()/stop_task()/logout()` | `start_sys()` / `stop_sys()` | ✅ `start_sys/stop_sys` |
| 状态查询 | 无 | `get_robot_state()` → `IDLE / MOVING / ESTOP` | ✅ 状态机 + 顶栏状态灯 |
| 数据反馈 | 无 | `get_kin_data()['actual_joint_pose' / 'actual_tcp_pose']` | ✅ 关节角 rad、TCP `{x,y,z,rx,ry,rz}` |
| 关节运动 | `movej(Q[key], T_JOINT)`（第 2 参数是时长） | `movej(p, a, v, t, r)`，返回 `motion_id`；`t>0` 时 a/v 失效 | ✅ 四参数语义完整 |
| 直线运动 | `movel(WP[key], T_LINEAR)` | `movel(p, a, v, t, r)`（a: m/s², v: m/s） | ✅ |
| 轨迹交融 | 无 | 第 5 参数 `r`（0–1，多段平滑） | ⚠️ 参数已接收，未参与规划（见 §3） |
| 等待完成 | 无（隐式等待） | `wait_move(id=None)` 阻塞至完成 | ✅ 队列 + Promise |
| 急停/停止 | 无急停按钮 | `stop_move()`；安全回路急停 → `ESTOP` | ✅ 急停按钮 = `stop_move + ESTOP`，复位 = `stop_sys→start_sys` |
| 暂停/恢复 | 无 | `pause_move()` / `resume_move()` | ✅ 接口已提供（UI 未挂按钮） |
| 夹爪 | `gripper_close()/open()`，单位 mm | `set_claw(force, amplitude)`，0–100 | ✅ 力度/开合度%；`get_claw()` 返回 `hold_on`、`weight` |
| 坐标系/单位 | 关节角用「度」 | 关节角 **rad**；位姿 **m/rad**，基座系 | ✅ 程序输出 rad，WP 为基座系 |
| 异常处理 | 无 try/except | 官方示例未强制；错误码形如 `[1002]` | ✅ 生成 `try/except/finally: stop_sys()`，仿真抛 `LebaiError(code,msg)` |
| 结束清理 | `logout()` | `stop_sys()` | ✅ `finally: stop_sys()` |

## 2. 修改方案与关键代码位置

| 文件 | 内容 |
| --- | --- |
| `js/lebai_sim.js`（新增） | `LebaiSim`：生命周期、状态机、运动队列（`enqueue` 返回 motion_id）、`movej/movel/wait_move/stop_move/pause_move/resume_move/get_motion_state`、`kinematics_forward/inverse/pose_trans/pose_add`、`set_claw/get_claw`、`get_di/set_do` 占位；`LebaiError` 错误码 1002/1003/1004/1005 |
| `js/robot.js:508` | `moveLMatrix()`：按目标位姿矩阵做直线插补；`stopMotion()` 与 `abortMotion` 支持运动中急停 |
| `js/planner.js:programFor()` | 生成官方风格 Python：`lebai_sdk.init()` → `connect` → `stop_sys` → `start_sys` → `try: movej/movel/wait_move/set_claw` → `except` → `finally: stop_sys()`；`Q`（rad）/`WP`（m,rad,基座系）/`HOME_Q` 字典 |
| `js/planner.js:parseProgram()` | 解析上述语法：常量替换、`Q/WP/HOME_Q` 字典、`movej/movel` 四参数、`set_claw`、`wait_move`、`try/except` 结构；不支持的循环会明确报错 |
| `js/app.js:runCode()` | 逐条执行解析结果，运动入队、`wait_move` 阻塞；异常统一捕获与复位；用时统计 |
| `js/app.js:pickAndPlace()` | **按钮抓放/自动分拣也走 `LebaiSim`**（`movej/movel/wait_move/set_claw`），与面板程序同源；`hintGrasp` 仅把已规划目标告诉夹爪，不改变 SDK 语义 |
| `js/app.js:toggleEstop()` | 急停/复位按钮（`stop_move` + `ESTOP` → `stop_sys`+`start_sys`） |
| `js/app.js:releaseObject()` | `set_claw` 张开时的工件释放，落在料盒内计入完成数 |

## 3. 仍与实机存在的差异（教学时需说明）

1. **动力学**：仿真为运动学插值（梯形速度近似），不含力矩、碰撞检测与安全限速；`a/v` 只影响时长。
2. **交融半径 r**：已解析但未用于路径平滑，多段连续运动仍是"到位再走"。
3. **网络与控制器**：`connect` 为本地仿真，不校验 IP、不做心跳与断线重连。
4. **`get_kin_data()` 字段**：只实现常用三项（actual/ target joint/tcp、flange_pose），未覆盖完整字典。
5. **IO/串口/场景任务**：`get_di/set_do` 为占位，`start_task/cancel_task` 未实现。
6. **速度单位**：`movej` 的 a/v 为 rad/s²、rad/s；`movel` 为 m/s²、m/s（与官方一致，学生最易混淆）。
