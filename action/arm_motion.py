"""UR5e 机械臂：末端（爪尖）从"起点"运动到"终点"，工具全程竖直朝下。

逆运动学（IK）用 GitHub 上的 mink 库：https://github.com/kevinzakka/mink
mink 的作用：你告诉它"末端要到哪里、姿态要怎样"，它帮你算出各关节该转多少。

场景来自 model/scene.xml：
    2 m × 2 m 木桌（桌面高 0.75 m）上立着这台 UR5e（已装好二指夹爪），
    桌上还有一个半径 0.1 m、质量 2 kg 的黄球和一个盘子。
机械臂底座在桌面上，所以整台手臂都"抬高"了 0.75 m —— 下面所有 z 坐标都是
从地面算起的世界坐标（桌面 = 0.75）。

程序分两步：
  第 1 步「算轨迹」：用 mink 沿着"起点 → 终点"的直线，一点点算出每个时刻的关节角；
  第 2 步「跑仿真」：把这些关节角发给 MuJoCo 的电机，看机械臂真的动起来。

用法（在项目根目录下打开终端执行）：
    macOS / Linux： .venv/bin/python  scripts/脚本名.py          # 不开窗口，跑完打印精度，存轨迹 CSV
                   .venv/bin/python  .venv/bin/mjpython scripts/脚本名.py --gui   # 开窗口
                   macOS 上开窗口必须用 mjpython，这是 MuJoCo 的硬性规定。
    Windows：       .venv/Scripts/python.exe scripts/脚本名.py --gui
                   Windows 不需要 mjpython，直接 python 就能开窗口。

你基本只需要改下面【① 你可以改的地方】里的几行数字，其余都不用动。

窗口里的快捷键（重要）：
    空格键   暂停 / 继续（MuJoCo 自带的 Run、Pause 按钮在"被动模式"下是灰的，
             因为我们这个程序是自己一步步驱动仿真的，所以暂停要按空格）
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import mujoco
import mink
import numpy as np

# ============================================================================
# ① 你可以改的地方（改这里的数字就够了）
# ============================================================================
# —— 夹爪的两个状态（路点里直接写 open / close 就行）——
# 开度单位是米（手指往里滑的距离）：
#   0      = 完全张开（两指内侧间距约 5.4 cm）
#   0.022  = 完全闭合（两指碰在一起）
#   0.005  = 刚好贴住直径 4.4 cm 的黄球
GRIPPER_OPEN  = 0.000   # ← 写 "open" 时用的开度
GRIPPER_CLOSE = 0.005   # ← 写 "close" 时用的开度（0.005 = 刚好贴住球；再大就是往里挤）
GRIPPER_NAMES = {"open": GRIPPER_OPEN, "close": GRIPPER_CLOSE}   # 想加别的名字：照这个格式加

# —— 末端要走的路：一行一个路点，走到哪儿顺手把夹爪改成什么状态 ——
# 每行 5 项：[x, y, z, 停留秒数, 夹爪状态]
#   x, y, z  : 世界坐标（米）
#   停留秒数 : 到这个点停多久，0 = 不停，直接平滑地穿过去
#   夹爪状态 : "open" / "close"，也可以直接写数字（米），写 None = 保持原样不动
# 执行顺序：先到点 → 停满上面的秒数 → 再把夹爪改成写的状态。
# 如果这个点的状态和上一个点一样，程序会跳过、不重复动作。
# 想加/删路点：直接加一行、删一行；没写停留的中间点会被平滑穿过（不会一顿一顿）。
# 如果某个点机械臂够不到，运行时会直接提示是第几个点、差了多少毫米。
WAYPOINTS = [
    [0.50, -0.05, 1.10,  0.5, "open"],    # ① 黄球上方：爪子张着
    [0.50,  -0.05, 0.762,  0.4, "close"],    # ② 黄球
    [0.52,  0.28, 0.867,  0.5, "close"],   # ③ 盘子上方
    [0.52,  0.28, 0.767,  0.5, "open"],   # ④ 放盘子里
    [0.52,  0.28, 0.867,  0.5, "open"],  # ③ 上移
    [0.50,  -0.05, 1.10,  0.5, "open"],    # ④ 回去
]

APPROACH_TIME = 5.0   # 初始 home 姿态 → 第一个路点 用多少秒
SEGMENT_TIME  = 6.0   # 每「相邻两个路点之间」用多少秒（点越多总时间越长；越大越慢越稳）
HOLD_TIME     = 1.0   # 走完全部路点后，再原地停多久（秒），方便你看清楚
# —— "末端始终朝下"这个限位 ——
KEEP_TOOL_DOWN  = True    # ← True = 末端全程竖直朝下；改成 False 就取消这个约束
SHOW_TOOL_ARROW = False    # ← True = 在窗口里用红色箭头画出"末端朝向"，方便看清是不是朝下
TOOL_DOWN_COST  = 10.0    # ← 姿态权重：越大越是"死死朝下"，越小越允许末端稍微歪一点
POSITION_COST   = 1.0     # ← 位置权重（一般不用改）
TILT_WARN_DEG   = 3.0     # ← 末端偏离"竖直向下"超过这个角度，就在终端里警告你

# —— 场景里两个物体的位置（照抄自 model/scene.xml；改位置时两边要一起改）——
BALL_POS  = np.array([0.50, -0.05, 0.772])   # ← 黄球球心：半径 0.022 m（直径 4.4 cm），质量 0.02 kg
PLATE_POS = np.array([0.52,  0.28, 0.757])   # ← 盘子中心：半径约 0.118 m，高约 2.5 cm
# 想把球挪个地方：① 改 model/scene.xml 里 <body name="ball" pos="...">；
#                  ② 这里的 BALL_POS 和上面的 WAYPOINTS 也要一起改成新坐标，否则会去空处找球。

# —— 坐标轴（在窗口里画出世界坐标的 X / Y / Z）——
SHOW_WORLD_AXES = True                      # ← True = 画三根箭头：红=X（前）绿=Y（左）蓝=Z（上）
# 画在桌子靠前左的一角（那里没东西挡着，看得最清楚）。
# 这三根箭头只表示"世界的 X/Y/Z 方向"，箭头的根部不一定是世界原点；
# 世界原点 (0,0,0) 是桌子正中央在地面上的那个点，正上方 0.75 m 就是桌面中心（机械臂底座的位置）。
AXIS_ORIGIN = np.array([-0.80, -0.80, 0.75]) # ← 想换个地方画就改这里（x, y, z）
AXIS_LENGTH = 0.30                          # ← 箭头长度（米），觉得太短太长就改这个

# —— IK 与电机参数 ——
IK_SOLVER        = "daqp"  # ← 二次规划求解器，可选 "daqp" / "quadprog" / "osqp"
MAX_JOINT_SPEED  = 3.0     # ← 规划时允许的关节最大速度（弧度/秒）
IK_DT            = 0.01    # ← 每隔 0.01 秒算一个轨迹点（100Hz，一般不用改）
PLAN_ITERS       = 30      # ← 每个轨迹点最多迭代几次 IK（越大越精确，也越慢）
TRACK_GAIN       = 8.0     # ← 电机跟随补偿的强度（伺服本身有滞后，用它把滞后补掉）
TRACK_LEAD_LIMIT = 0.4     # ← 补偿量上限（弧度），防止补偿过头

# —— 文件路径（都相对项目根目录 work4）——
XML_PATH    = "model/scene.xml"    # 仿真用的场景（含桌子/球/盘子），里面 <include> 了 ur5e.xml
IK_XML_PATH = "model/ur5e.xml"     # 只给 IK 用的纯机械臂模型（不含球和盘子，免得 IK 去动它们）
SITE_NAME   = "tool_site"          # 末端那个点在 ur5e.xml 里的名字（爪尖）
CSV_PATH    = "out/trajectory.csv" # 跑完把轨迹存到这里

# —— 打开窗口时的默认视角（觉得角度不好看就改这几个数）——
CAM_LOOKAT    = [0.20, 0.0, 1.00]  # 镜头看哪儿（世界坐标）
CAM_DISTANCE  = 2.4                # 镜头离看点多远，越大画面越远
CAM_AZIMUTH   = 130                # 镜头绕竖直轴的角度（度）
CAM_ELEVATION = -22                # 镜头俯视角度（度），负数是往下看

# 机械臂 6 个关节的名字（和 ur5e.xml 里一致，一般不用改）
ARM_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
# 夹爪两个手指关节的名字
FINGER_JOINTS = ["right_finger_joint", "left_finger_joint"]

HOME_SETTLE_TIME = 1.0   # 开始前先让机械臂在 home 姿态站稳这么多秒（秒）

# ============================================================================
# ② 下面是程序本体，一般不用看；想学原理再往下读
# ============================================================================
# 脚本可能被放在 运行器/scripts/（开发目录）或分发包的 scripts/：
# 从脚本所在目录往上找含 model/ur5e.xml 的那一层当项目根。
ROOT = next((p for p in Path(__file__).resolve().parents
             if (p / "model" / "ur5e.xml").is_file()),
            Path(__file__).resolve().parent.parent)
DOWN_DIRECTION = np.array([0.0, 0.0, -1.0])


def smoothstep(s):
    """平滑插值曲线：起步和停止的速度都是 0，看起来不突兀（s = 3u² - 2u³）。"""
    s = np.clip(s, 0.0, 1.0)
    return s * s * (3.0 - 2.0 * s)


def add_arrow(scn, pos, direction, length, color, shaft=0.006):
    """往 MuJoCo 窗口的"用户图层"里加一支箭头（用来画坐标轴）。

    scn       : viewer.user_scn，窗口的用户图层
    pos       : 箭头尾部位置（世界坐标）
    direction : 箭头指向（世界坐标下的方向，不必归一化）
    """
    if scn.ngeom >= scn.maxgeom:
        return None
    geom = scn.geoms[scn.ngeom]
    scn.ngeom += 1
    # 箭头模型默认沿自己的 +z 生长，所以先把 +z 转到要指的方向
    R = rotation_between(np.array([0.0, 0.0, 1.0]), np.asarray(direction, dtype=float))
    mujoco.mjv_initGeom(
        geom, mujoco.mjtGeom.mjGEOM_ARROW,
        np.asarray([shaft, shaft, length], dtype=np.float64),
        np.asarray(pos, dtype=np.float64),
        np.asarray(R, dtype=np.float64).flatten(),
        np.asarray(color, dtype=np.float32),
    )
    return geom


def rotation_between(a, b):
    """求"把方向 a 转到方向 b"的旋转矩阵（罗德里格斯公式）。"""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    c = float(np.dot(a, b))
    if c > 1.0 - 1e-9:
        return np.eye(3)
    if c < -1.0 + 1e-9:
        axis = np.cross(a, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, np.array([0.0, 1.0, 0.0]))
        axis = axis / np.linalg.norm(axis)
        return 2.0 * np.outer(axis, axis) - np.eye(3)
    v = np.cross(a, b)
    s = np.linalg.norm(v)
    vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / (s * s))


def tool_down_rotation(model, scratch_data):
    """算出"工具竖直朝下"对应的末端姿态矩阵 R_target。

    末端（site）自己的 +z 轴就是工具的指向。这里先看 home 姿态下它指向哪儿，
    再把它旋转到"世界竖直向下"，同时尽量少转（保持 x/y 朝向自然）。
    """
    mujoco.mj_resetDataKeyframe(model, scratch_data, 0)
    mujoco.mj_forward(model, scratch_data)
    R = scratch_data.site(SITE_NAME).xmat.reshape(3, 3).copy()
    axis = R[:, 2]                      # 末端 +z 在世界里的方向
    R_fix = rotation_between(axis, DOWN_DIRECTION)
    return R_fix @ R


class ArmIK:
    """把 mink 的逆运动学包装成一个"给目标点 → 返回各关节角"的小工具。"""

    def __init__(self, model):
        self.config = mink.Configuration(model)
        q_home = model.key_qpos[0].copy() if model.nkey > 0 else model.qpos0.copy()
        self.config.update(q=q_home)

        # 位置任务 + 姿态任务（姿态任务负责"末端朝下"）
        self.frame_task = mink.FrameTask(
            frame_name=SITE_NAME,
            frame_type="site",
            position_cost=POSITION_COST,
            orientation_cost=TOOL_DOWN_COST if KEEP_TOOL_DOWN else 0.0,
            lm_damping=1.0,
        )
        # 姿态正则：让 IK 尽量选"离 home 不太远"的那个解，避免手臂乱翻
        self.posture_task = mink.PostureTask(model, cost=1e-4)
        self.posture_task.set_target(q_home)
        self.tasks = [self.frame_task, self.posture_task]

        # 硬约束：关节限位（模型里写死的） + 每个关节的速度上限
        # 只给 6 个机械臂关节设速度上限；夹爪手指与末端位置无关，由姿态正则自然保持不动
        velocity = {name: MAX_JOINT_SPEED for name in ARM_JOINTS}
        self.limits = [mink.ConfigurationLimit(model), mink.VelocityLimit(model, velocity)]

    def reset(self, q):
        """把 IK 的"当前姿态"设成 q。"""
        self.config.update(q=np.asarray(q, dtype=float))

    def ee_position(self):
        return self.config.get_transform_frame_to_world(SITE_NAME, "site").translation()

    def go_to(self, target_pos, target_rot, iters=PLAN_ITERS):
        """让末端走到 target_pos、姿态转到 target_rot。返回 (关节角, 位置误差[米])。"""
        target_pos = np.asarray(target_pos, dtype=float)
        T = mink.SE3.from_rotation_and_translation(
            rotation=mink.SO3.from_matrix(np.asarray(target_rot, dtype=float)),
            translation=target_pos,
        )
        self.frame_task.set_target(T)
        err = np.inf
        for _ in range(iters):
            velocity = mink.solve_ik(self.config, self.tasks, IK_DT, IK_SOLVER, limits=self.limits)
            self.config.integrate_inplace(velocity, IK_DT)
            err = float(np.linalg.norm(self.ee_position() - target_pos))
            if err < 1e-5:
                break
        return self.config.q.copy(), err


def plan_segment(ik, start_pos, end_pos, duration, target_rot):
    """规划一段"从 start_pos 直线走到 end_pos"的轨迹（用于 home → 第一个路点）。

    每个轨迹点都让 IK 迭代到收敛（而不是只算一步），这样规划出来的关节角才
    真正对应末端位置 —— 机械臂的伺服电机有点"肉"，只给一步的 IK 结果会跑偏。
    返回 (关节角序列, 最大 IK 误差, 每个点对应的末端目标位置)。
    """
    n = max(2, int(round(duration / IK_DT)))
    qs, errs, poss = [], [], []
    for k in range(n + 1):
        s = smoothstep(k / n)
        pos = start_pos + (end_pos - start_pos) * s
        q, err = ik.go_to(pos, target_rot)
        qs.append(q)
        errs.append(err)
        poss.append(pos)
    return np.array(qs), max(errs), np.array(poss)


def parse_waypoints(waypoints):
    """把每个路点解析成 (位置, 停留秒数, 夹爪状态)。

    每行可以写 3、4 或 5 项：[x, y, z] / [x, y, z, 停留] / [x, y, z, 停留, 夹爪]。
    夹爪状态可以写 "open" / "close"，也可以直接写数字（米），写 None = 保持不动。
    """
    pts, dwells, grips = [], [], []
    for w in waypoints:
        w = list(w)
        if len(w) < 3:
            raise ValueError(f"路点 {w} 至少要写 x, y, z 三个数")
        pts.append(np.array([float(w[0]), float(w[1]), float(w[2])]))
        dwells.append(float(w[3]) if len(w) > 3 and w[3] is not None else 0.0)
        grip = None
        if len(w) > 4 and w[4] is not None:
            g = w[4]
            if isinstance(g, str):
                key = g.strip().lower()
                if key not in GRIPPER_NAMES:
                    raise ValueError(f"夹爪状态只能写 {list(GRIPPER_NAMES)} 或者数字，收到 {g!r}")
                grip = GRIPPER_NAMES[key]
            else:
                grip = float(g)
        grips.append(grip)
    return pts, dwells, grips


def plan_waypoints(ik, waypoints, target_rot):
    """按顺序走过多个路点；停稳后按路点里写的状态改变夹爪。

    一段一段地走：每段用 SEGMENT_TIME 秒；位置用三次曲线插值，速度在路点处接得上，
    所以中间的路点会被"平滑穿过"（不停），只有写了停留秒数的点才真的停住。
    每个路点按 [x, y, z, 停留秒数, 夹爪状态] 执行：先到点 → 停满秒数 → 再改夹爪。
    返回 (关节角序列, 最大 IK 误差, 目标位置序列, 到点样本序号, 各路点 IK 误差, 每样本夹爪开度)。
    """
    pts, dwells, grips = parse_waypoints(waypoints)
    if len(pts) == 1:
        pts = [pts[0], pts[0]]
        dwells = [dwells[0], 0.0]
        grips = [grips[0], grips[0]]
    n_seg = max(2, int(round(SEGMENT_TIME / IK_DT)))

    # 夹爪的"当前状态"：从第一个路点写的状态开始（没写就默认张开）
    cur_grip = grips[0] if grips[0] is not None else GRIPPER_OPEN

    # 先给每个路点算一个"经过速度"：要停留的点速度 = 0（会停稳），
    # 中间路过不停的点保留速度（这样它会被平滑地"穿过"，不会一顿一顿）。
    vel = [np.zeros(3) for _ in range(len(pts))]
    for j in range(1, len(pts) - 1):
        if dwells[j] > 0:
            continue
        # Catmull-Rom 风格的切线：取前后两个路点的连线方向，
        # 长度约为该处"一段路"的长度，保证速度连续、拐角自然圆滑（不会冲出去）。
        tangent = (pts[j + 1] - pts[j - 1]) * 0.5
        d_prev = float(np.linalg.norm(pts[j] - pts[j - 1]))
        d_next = float(np.linalg.norm(pts[j + 1] - pts[j]))
        limit = 0.9 * min(d_prev, d_next)
        norm = float(np.linalg.norm(tangent))
        if norm > 1e-9:
            vel[j] = tangent / norm * min(norm, limit)

    positions, arrive, grip_at = [], [], []
    for j in range(len(pts) - 1):
        p0, p1 = pts[j], pts[j + 1]
        v0, v1 = vel[j], vel[j + 1]
        for k in range(n_seg + 1):
            if j > 0 and k == 0:
                continue                     # 上一段的终点就是这个起点，别重复
            u = k / n_seg
            # 三次 Hermite 插值：位置对得上，速度也接得上（这就是"平滑"的关键）
            h00 = 2 * u ** 3 - 3 * u ** 2 + 1
            h10 = u ** 3 - 2 * u ** 2 + u
            h01 = -2 * u ** 3 + 3 * u ** 2
            h11 = u ** 3 - u ** 2
            positions.append(h00 * p0 + h10 * v0 + h01 * p1 + h11 * v1)
            grip_at.append(cur_grip)
        arrive.append(len(positions) - 1)    # 这一刻到达第 j+1 个路点
        for _ in range(int(round(dwells[j + 1] / IK_DT))):
            positions.append(pts[j + 1])     # 停留（这期间夹爪先不动）
            grip_at.append(cur_grip)
        # 停留结束：把夹爪改成这个路点写的状态（和当前一样就跳过）
        if grips[j + 1] is not None and abs(grips[j + 1] - cur_grip) > 1e-9:
            cur_grip = grips[j + 1]

    # 最后一个路点的夹爪动作：后面还得有样本它才执行得了，补一小段静止
    if abs(cur_grip - grip_at[-1]) > 1e-9:
        for _ in range(max(1, int(round(0.4 / IK_DT)))):
            positions.append(pts[-1])
            grip_at.append(cur_grip)

    qs, errs = [], []
    for pos in positions:
        q, err = ik.go_to(pos, target_rot)
        qs.append(q)
        errs.append(err)
    err_at_wp = [errs[i] for i in arrive]        # 每个路点自己的 IK 误差（够不够得到）
    return np.array(qs), max(errs), np.array(positions), arrive, err_at_wp, np.array(grip_at)


def main():
    gui = "--gui" in sys.argv
    # macOS 上开窗口必须跑在 mjpython 里，否则会崩；顺手给个提示
    if gui and sys.platform == "darwin" and "MJPYTHON_BIN" not in os.environ:
        print("提示：macOS 上请用 .venv/bin/mjpython 运行（或双击 run_gui.command）。")
        return

    xml_file = ROOT / XML_PATH
    if not xml_file.exists():
        print(f"找不到场景文件 {xml_file}，请确认 model/ 目录完整。")
        return

    # ---------------- 加载模型 ----------------
    model = mujoco.MjModel.from_xml_path(str(xml_file))       # 仿真用（含球、盘子）
    data = mujoco.MjData(model)
    ik_model = mujoco.MjModel.from_xml_path(str(ROOT / IK_XML_PATH))  # IK 用（只有机械臂）
    scratch = mujoco.MjData(model)

    R_DOWN = tool_down_rotation(model, scratch)  # "工具朝下"的姿态矩阵
    print(f"末端朝下时，工具指向（世界坐标）{np.round(R_DOWN[:, 2], 3)}")

    # 关节名 → 在 qpos 里的位置、以及对应的电机编号
    arm_qadr = [int(model.joint(n).qposadr[0]) for n in ARM_JOINTS]
    fing_qadr = [int(model.joint(n).qposadr[0]) for n in FINGER_JOINTS]
    arm_act, fing_act = [], []
    for name in ARM_JOINTS + FINGER_JOINTS:
        jid = int(model.joint(name).id)
        found = None
        for i in range(model.nu):
            if model.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT and int(model.actuator_trnid[i, 0]) == jid:
                found = i
                break
        if found is None:
            print(f"⚠ 关节 {name} 没有对应的电机，请检查 ur5e.xml")
            return
        (arm_act if name in ARM_JOINTS else fing_act).append(found)

    # ---------------- 先在 home 姿态站稳 ----------------
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.ctrl[:] = 0.0
    for i in arm_act:
        data.ctrl[i] = data.qpos[arm_qadr[arm_act.index(i)]]
    for i in fing_act:
        data.ctrl[i] = GRIPPER_OPEN            # 先张开，等会儿按路点指令动
    for _ in range(int(round(HOME_SETTLE_TIME / model.opt.timestep))):
        mujoco.mj_step(model, data)

    q_now = data.qpos.copy()
    ik = ArmIK(ik_model)
    # 把 IK 的初始姿态设成机械臂此刻的真实姿态（夹爪手指用 IK 模型里的当前值）
    q_ik0 = ik_model.qpos0.copy()
    if ik_model.nkey > 0:
        q_ik0 = ik_model.key_qpos[0].copy()
    for name in ARM_JOINTS:
        q_ik0[int(ik_model.joint(name).qposadr[0])] = data.qpos[int(model.joint(name).qposadr[0])]
    for name in FINGER_JOINTS:
        q_ik0[int(ik_model.joint(name).qposadr[0])] = data.qpos[int(model.joint(name).qposadr[0])]
    ik.reset(q_ik0)

    # ---------------- 第 1 步：规划轨迹 ----------------
    print("\n正在用 mink 规划轨迹（这一步不显示画面）…")
    t0 = time.time()
    ee0 = ik.ee_position()
    wp_pts, wp_dwell, wp_grip = parse_waypoints(WAYPOINTS)
    grip0 = wp_grip[0] if wp_grip[0] is not None else GRIPPER_OPEN
    print(f"要依次经过 {len(wp_pts)} 个路点，每段 {SEGMENT_TIME:.1f} 秒")
    q_approach, err_a, pos_approach = plan_segment(ik, ee0, wp_pts[0], APPROACH_TIME, R_DOWN)
    q_move, err_b, pos_move, arrive, wp_err, grip_path = plan_waypoints(ik, WAYPOINTS, R_DOWN)
    n_hold = int(round(HOLD_TIME / IK_DT))
    q_hold = np.repeat(q_move[-1][None, :], n_hold + 1, axis=0)
    q_plan = np.vstack([q_approach, q_move[1:], q_hold[1:]])
    # 每个规划点对应的末端目标位置（用来算"实际走了多偏"）
    plan_pos = np.vstack([pos_approach, pos_move[1:],
                          np.repeat(pos_move[-1][None, :], n_hold, axis=0)])
    # 每个规划点对应的夹爪开度（接近段保持初始状态，最后停留保持不变）
    grip_plan = np.concatenate([np.full(len(q_approach), grip0), grip_path[1:],
                                np.full(n_hold, grip_path[-1])])
    plan_err = max(err_a, err_b)

    arm_idx = [int(ik_model.joint(n).qposadr[0]) for n in ARM_JOINTS]
    q_arm_plan = q_plan[:, arm_idx]
    dq = np.abs(np.diff(q_arm_plan, axis=0)).max() / IK_DT
    print(f"规划完成：{len(q_plan)} 个点，用了 {time.time() - t0:.2f} 秒")
    print(f"规划时 IK 的最大位置误差 {plan_err * 1000:.2f} mm，最大关节速度 {dq:.2f} 弧度/秒")
    # 下面是「到达每个路点时的进度」，写 GRIPPER_TIMELINE 时照着填就很准
    n_approach = len(pos_approach) - 1
    for i, idx in enumerate(arrive, start=1):
        prog = (n_approach + idx) / (len(q_plan) - 1)
        extra = ""
        if wp_err[i - 1] > 0.005:      # 5 mm 以上就算"没够到"
            extra = f"  ⚠ 这个点机械臂够不到（差 {wp_err[i - 1] * 1000:.0f} mm），请往底座方向挪近"
        print(f"  路点 {i + 1} 到达时进度 {prog:.2f}{extra}")
    def grip_name(g):
        if abs(g - GRIPPER_OPEN) < 1e-9:
            return "张开"
        if abs(g - GRIPPER_CLOSE) < 1e-9:
            return "闭合"
        return f"开度 {g:.3f}"

    prev_grip = None
    for i, g in enumerate(wp_grip, start=1):
        if g is None or (prev_grip is not None and abs(g - prev_grip) < 1e-9):
            continue                      # 没写 / 和上一点一样 → 不动，也不打印
        prev_grip = g
        when = "出发时" if i == 1 else f"路点 {i} 停稳后"
        print(f"  夹爪：{when} →「{grip_name(g)}」")
    if dq > MAX_JOINT_SPEED * 1.05:
        print("⚠ 速度偏大，动作可能跟不上：把 DURATION / APPROACH_TIME 改大一些会更稳。")

    # ---------------- 第 2 步：跑仿真，让机械臂真的动起来 ----------------
    viewer = None
    arrow = None
    paused = [False]                                   # 空格键暂停开关
    pause_stamp = [0.0]

    def on_key(keycode):
        """窗口里的键盘回调：按空格 = 暂停 / 继续。"""
        if keycode == 32:                              # 32 = 空格键
            paused[0] = not paused[0]
            if paused[0]:
                pause_stamp[0] = time.time()
                print("⏸  已暂停（再按一次空格继续；暂停时可以用鼠标拖动视角）")
            else:
                print("▶  继续运动")

    if gui:
        from mujoco import viewer as mj_viewer          # 注意：别写 import mujoco.viewer
        # launch_passive = "被动模式"：窗口只负责显示，仿真由我们的 Python 循环推进。
        # 好处是机械臂的动作完全由程序控制（mink 规划出来的轨迹），
        # 代价是窗口自带的 Run / Pause 按钮会变灰（那是给"托管模式"用的），所以这里自己实现空格暂停。
        viewer = mj_viewer.launch_passive(model, data, key_callback=on_key)
        viewer.cam.lookat[:] = CAM_LOOKAT
        viewer.cam.distance = CAM_DISTANCE
        viewer.cam.azimuth = CAM_AZIMUTH
        viewer.cam.elevation = CAM_ELEVATION
        if SHOW_WORLD_AXES:
            # 三根坐标轴：红 = X（向"前"），绿 = Y（向"左"），蓝 = Z（向上）
            for axis_dir, axis_color, axis_name in [
                (np.array([1.0, 0.0, 0.0]), [1.0, 0.30, 0.30, 0.95], "X"),
                (np.array([0.0, 1.0, 0.0]), [0.30, 1.0, 0.35, 0.95], "Y"),
                (np.array([0.0, 0.0, 1.0]), [0.35, 0.45, 1.0, 0.95], "Z"),
            ]:
                add_arrow(viewer.user_scn, AXIS_ORIGIN, axis_dir, AXIS_LENGTH, axis_color, shaft=0.010)
                print(f"已画出坐标轴 {axis_name} 轴（长度 {AXIS_LENGTH} m）")
        if SHOW_TOOL_ARROW:
            # 末端朝向红色箭头：会跟着爪尖实时更新（在下面的循环里改它的位置）
            arrow = add_arrow(
                viewer.user_scn, data.site(SITE_NAME).xpos, np.array([0.0, 0.0, 1.0]), 0.14,
                [1.0, 0.25, 0.25, 0.9], shaft=0.008,
            )

    # 让机身先回到规划时的起始姿态，再逐点下发指令
    for i, adr in enumerate(arm_qadr):
        data.qpos[adr] = q_arm_plan[0, i]
    data.qpos[fing_qadr] = grip_plan[0]
    mujoco.mj_forward(model, data)

    substeps = max(1, int(round(IK_DT / model.opt.timestep)))
    q_int = np.zeros(len(arm_qadr))                    # 积分补偿量
    log = []
    site = data.site(SITE_NAME)
    t_wall0 = time.time()
    last_sync = 0.0
    for k in range(len(q_arm_plan)):
        if paused[0]:
            # 暂停：不推进物理，只刷新画面（这样才能用鼠标转视角看细节）
            if viewer is not None:
                viewer.sync()
            time.sleep(1.0 / 60.0)
            continue

        q_ref = q_arm_plan[k]
        q_cur = data.qpos[arm_qadr]
        # 位置伺服本身有滞后（夹爪还有重量），用积分补偿把滞后补掉
        if pause_stamp[0] > 0.0:
            # 刚从暂停恢复：把"实时配速"的基准时间往后挪，避免恢复瞬间快进
            t_wall0 += time.time() - pause_stamp[0]
            pause_stamp[0] = 0.0
        q_int = np.clip(q_int + TRACK_GAIN * (q_ref - q_cur) * IK_DT,
                        -TRACK_LEAD_LIMIT, TRACK_LEAD_LIMIT)
        for i, act in enumerate(arm_act):
            data.ctrl[act] = q_ref[i] + q_int[i]
        grip = grip_plan[k]
        for act in fing_act:
            data.ctrl[act] = grip

        for _ in range(substeps):
            mujoco.mj_step(model, data)

        tilt = float(np.degrees(np.arccos(np.clip(np.dot(site.xmat.reshape(3, 3)[:, 2], DOWN_DIRECTION), -1.0, 1.0))))
        log.append(np.concatenate([[data.time], q_ref, site.xpos.copy(), [tilt], data.qpos.copy()]))

        if viewer is not None:
            now = time.time()
            if now - last_sync > 1.0 / 60.0:
                if arrow is not None:
                    arrow.pos[:] = site.xpos
                    arrow.mat[:] = site.xmat.reshape(3, 3)
                viewer.sync()
                last_sync = now
            sleep = t_wall0 + data.time - time.time()      # 让画面和真实时间同步
            if sleep > 0:
                time.sleep(sleep)

    # ---------------- 结果统计 ----------------
    arr = np.array(log)
    nq = len(arm_qadr)
    des = arr[:, 1:1 + nq]
    ee = arr[:, 1 + nq:1 + nq + 3]
    tilt_all = arr[:, 1 + nq + 3]
    arm_q = arr[:, 1 + nq + 4: 1 + nq + 4 + model.nq]

    # 沿路径的偏差：实际末端 vs 规划时每个点想要的末端位置
    track_err = np.linalg.norm(ee - plan_pos[:len(ee)], axis=1)

    print("\n================ 结果 ================")
    print(f"末端最终位置 {np.round(site.xpos, 4)}，与最后一个路点误差 "
          f"{np.linalg.norm(wp_pts[-1] - site.xpos) * 1000:.1f} mm")
    print(f"运动中沿路径的最大偏差 {track_err.max() * 1000:.1f} mm")
    if KEEP_TOOL_DOWN:
        print(f"末端偏离'竖直向下'的最大角度 {tilt_all.max():.2f} 度")
        if tilt_all.max() > TILT_WARN_DEG:
            print(f"⚠ 超过 {TILT_WARN_DEG} 度：可以把 TOOL_DOWN_COST 调大，或把动作放慢。")
        else:
            print("✓ 末端全程基本竖直朝下")
    print(f"最终 6 个关节角（弧度）{np.round(data.qpos[arm_qadr], 3)}")
    print(f"夹爪最终开度 {data.ctrl[fing_act[0]]:.3f}（0 = 全开，0.022 = 全闭），"
          f"两根手指实际位置 {np.round(data.qpos[fing_qadr], 4)}")

    out_csv = ROOT / CSV_PATH
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    header = ("time," + ",".join(f"des_q{i}" for i in range(nq)) + ",ee_x,ee_y,ee_z,tilt_deg,"
              + ",".join(f"q{i}" for i in range(model.nq)))
    np.savetxt(out_csv, arr, delimiter=",", header=header, comments="")
    print(f"轨迹已保存到 {CSV_PATH}（{len(arr)} 行）")

    # ---------------- 窗口留着不关，你自己点红叉退出 ----------------
    if viewer is not None:
        print("机械臂已停在终点，窗口保持打开，看够了直接关掉窗口。")
        while viewer.is_running():
            viewer.sync()
            time.sleep(1.0 / 60.0)
        viewer.__exit__(None, None, None)


if __name__ == "__main__":
    main()
