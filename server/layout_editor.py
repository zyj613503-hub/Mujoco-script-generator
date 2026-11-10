#!/usr/bin/env python3
"""MuJoCo 交互式布置窗口：拖动球 / 盘子摆位，按 S 保存。

为什么必须用 mjpython：
    macOS 上 MuJoCo 开窗要求 Cocoa 主线程空闲，普通 python 会直接报错退出。
    启动方式（由 app.py 的 /api/layout/launch 自动完成）：
        .venv/bin/python .venv/bin/mjpython 运行器/server/layout_editor.py \
            --scene model/scene.xml --out 运行器/out/layout.json --session <id>

窗口里的操作：
    * Ctrl + 右键拖动     对选中的物体施加外力（推开 / 拖动它）
    * 双击物体             选中该物体，之后可以直接拖动
    * S                    保存当前布置
    * 关闭窗口             也会自动保存

保存结果：
    运行器/out/layout.json
        {sessionId, savedAt, bodies:[{name,pos}], scene, savedScene}
    运行器/out/current_layout.json
        {savedAt, scene, savedScene, bodies}（覆盖式，给副界面恢复当前布置）
    model/scenes/scene_layout_<YYYYmmdd-HHMMSS>.xml
        复制原场景文本，只把 free body（球、盘子）的 pos 改成窗口里的新位置，
        并把 include / asset 的相对路径改成相对 model/scenes/ 的写法。

自检（不开窗口）：
    .venv/bin/python 运行器/server/layout_editor.py --scene model/scene.xml --check
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
import traceback
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve()
SERVER_DIR = HERE.parent
RUNNER_DIR = HERE.parents[1]


def _find_root(start):
    """向上找含 model/ur5e.xml 的目录（兼容 work4/运行器 与扁平分发包两种布局）。"""
    for parent in [start, *start.parents]:
        if (parent / "model" / "ur5e.xml").is_file():
            return parent
    return start.parent


WORK4_DIR = _find_root(RUNNER_DIR)

if str(SERVER_DIR) not in sys.path:
    sys.path.insert(0, str(SERVER_DIR))

import scene as scenelib  # noqa: E402  （同目录的场景 XML 解析）

FREE_BODY_FALLBACK = ("ball", "plate")
CURRENT_LAYOUT_NAME = "current_layout.json"


# ---------------------------------------------------------------------------
# 场景文本处理
# ---------------------------------------------------------------------------
def _set_body_pos(text, name, pos):
    """在场景文本里改 <body name="X" ...> 的 pos 属性，返回 (新文本, 是否命中)。"""
    tag_re = re.compile(r'<body\b[^>]*\bname="%s"[^>]*>' % re.escape(name))
    match = tag_re.search(text)
    if match is None:
        return text, False
    tag = match.group(0)
    value = 'pos="%.4f %.4f %.4f"' % (pos[0], pos[1], pos[2])
    new_tag, count = re.subn(r'pos="[^"]*"', value, tag, count=1)
    if count == 0:
        new_tag = tag[:-1] + " " + value + ">"
    return text[:match.start()] + new_tag + text[match.end():], True


ASSET_DIR = WORK4_DIR / "model" / "assets"


def _rebase_refs(text):
    """把 file="assets/..." / file="ur5e.xml" 改成相对新场景目录的写法。"""
    # <include> 一律加上前缀（include 的文件和场景不在同一目录）
    text = re.sub(
        r'(<include\b[^>]*\bfile=")([^"]+)"',
        lambda m: m.group(1) + (
            m.group(2) if m.group(2).startswith((".", "/")) else "../" + m.group(2)) + '"',
        text)

    # 其余带目录的 file="assets/..."；光秃秃的 mesh 名交给 <compiler meshdir>
    def fix(match):
        value = match.group(1)
        if value.startswith(("/", "./", "../")) or "/" not in value:
            return match.group(0)
        return 'file="../%s"' % value

    return re.sub(r'file="([^"]+)"', fix, text)


def _inject_compiler(text, scene_dir):
    """把 ur5e.xml 的资产目录重新指到 assets/。

    MuJoCo 只认「顶层模型文件目录 + meshdir」，把场景挪到 model/scenes/ 后，
    被 <include> 进来的 ur5e.xml 里的 <mesh file="xxx.obj"> 会去找
    model/scenes/xxx.obj 而失败。这里在 <include> 之后补一个 <compiler>，
    用相对路径指回 model/assets/；同时保留 ur5e.xml 依赖的 radian / autolimits。
    """
    if "<include" not in text:
        return text
    try:
        meshdir = os.path.relpath(ASSET_DIR, Path(scene_dir)).replace(os.sep, "/")
    except ValueError:
        meshdir = "../assets"
    compiler = ('<compiler angle="radian" autolimits="true" meshdir="%s"/>'
                % meshdir)
    matches = list(re.finditer(r"<include\b[^>]*/>", text))
    if not matches:
        return text
    last = matches[-1]
    return text[:last.end()] + "\n  " + compiler + text[last.end():]


def _free_bodies(model, mujoco):
    """找出带 free joint 的 body（球、盘子），返回 [(name, body_id, qpos_addr)]。"""
    result = []
    for body_id in range(model.nbody):
        jnt_num = int(model.body_jntnum[body_id])
        if jnt_num <= 0:
            continue
        jnt_adr = int(model.body_jntadr[body_id])
        for offset in range(jnt_num):
            jid = jnt_adr + offset
            if int(model.jnt_type[jid]) == int(mujoco.mjtJoint.mjJNT_FREE):
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
                result.append((name or "body%d" % body_id, body_id,
                               int(model.jnt_qposadr[jid])))
                break
    return result


def collect_bodies(model, data, mujoco):
    """当前自由物体的位置（按场景顺序）。"""
    bodies = []
    for name, _body_id, qadr in _free_bodies(model, mujoco):
        pos = [round(float(v), 5) for v in data.qpos[qadr:qadr + 3]]
        bodies.append({"name": name, "pos": pos})
    return bodies


def save_layout(model, data, mujoco, *, scene_rel, scene_text, scene_dir,
                out_json, session_id, target_name, write_current=True):
    """写 scene_layout_*.xml + out/layout.json（+ current_layout.json），返回保存摘要。"""
    bodies = collect_bodies(model, data, mujoco)

    scene_dir = Path(scene_dir)
    scene_dir.mkdir(parents=True, exist_ok=True)
    target = scene_dir / target_name

    text = scene_text
    source_dir = (WORK4_DIR / scene_rel).resolve().parent
    if source_dir != scene_dir.resolve():
        text = _rebase_refs(text)
        text = _inject_compiler(text, scene_dir)
    missed = []
    for entry in bodies:
        text, ok = _set_body_pos(text, entry["name"], entry["pos"])
        if not ok:
            missed.append(entry["name"])
    target.write_text(text, encoding="utf-8")

    # 回读校验：用与俯视图同一套 XML 解析比较 body 的绝对 pos。
    # 写盘时 pos 保留 4 位小数、qpos 保留 5 位，差 1e-5 属于正常，容差取 1e-4。
    verify_error = None
    try:
        positions = scenelib.body_positions(target)
        for entry in bodies:
            got = positions.get(entry["name"])
            if got is None:
                verify_error = "生成场景里找不到 body %s" % entry["name"]
                break
            if any(abs(a - b) > 1e-4 for a, b in zip(got, entry["pos"])):
                verify_error = "生成场景里 %s 的位置与保存值不一致" % entry["name"]
                break
    except Exception as exc:  # noqa: BLE001
        verify_error = "生成场景无法重新解析：%s" % exc

    payload = {
        "sessionId": session_id,
        "savedAt": datetime.now().isoformat(timespec="seconds"),
        "bodies": bodies,
        "scene": scene_rel,
        "savedScene": _rel_posix(target),
    }
    if verify_error:
        payload["verifyError"] = verify_error
    if missed:
        payload["unmatched"] = missed

    out_json = Path(out_json)
    out_json.parent.mkdir(parents=True, exist_ok=True)
    tmp = out_json.with_suffix(out_json.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(tmp, out_json)

    # 「当前布置」单独存一份、覆盖式写入，给副界面「加载已有脚本」恢复场景用。
    # 自检模式（--check）不写，免得把临时布局覆盖到真实结果上。
    if write_current:
        current = out_json.parent / CURRENT_LAYOUT_NAME
        current_tmp = current.with_suffix(current.suffix + ".tmp")
        current_payload = {
            "savedAt": payload["savedAt"],
            "scene": payload["scene"],
            "savedScene": payload["savedScene"],
            "bodies": payload["bodies"],
        }
        current_tmp.write_text(
            json.dumps(current_payload, ensure_ascii=False, indent=2),
            encoding="utf-8")
        os.replace(current_tmp, current)
    return payload, target


def _rel_posix(path):
    try:
        return Path(path).resolve().relative_to(WORK4_DIR).as_posix()
    except ValueError:
        return Path(path).as_posix()


# ---------------------------------------------------------------------------
# --check：无窗口自检
# ---------------------------------------------------------------------------
def run_check(scene_rel, out_json, session_id):
    import tempfile

    import mujoco

    scene_path = (WORK4_DIR / scene_rel).resolve()
    scene_text = scene_path.read_text(encoding="utf-8")

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    if model.nkey:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    free = _free_bodies(model, mujoco)
    print("场景：%s" % scene_rel)
    print("自由物体：%s" % ", ".join(name for name, _b, _q in free))
    if not free:
        print("失败：场景里没有 free joint 物体", file=sys.stderr)
        return 2

    # 挪一下第一个自由物体，模拟拖动
    name0, _body0, qadr0 = free[0]
    data.qpos[qadr0] += 0.05
    data.qpos[qadr0 + 1] += 0.03
    mujoco.mj_forward(model, data)

    # 临时目录放在 model/ 下：这样生成场景里的 ../ur5e.xml、../assets/... 都能解析，
    # 又不会把测试文件留在 model/scenes/ 里。跑完删掉。
    tmpdir = WORK4_DIR / "model" / (".layout-check-%d" % os.getpid())
    try:
        payload, target = save_layout(
            model, data, mujoco,
            scene_rel=scene_rel, scene_text=scene_text, scene_dir=tmpdir,
            out_json=Path(out_json).with_suffix(".check.json"), session_id=session_id,
            target_name="scene_layout_check.xml", write_current=False)
        saved = json.loads(
            Path(out_json).with_suffix(".check.json").read_text(encoding="utf-8"))
        assert saved["sessionId"] == session_id
        assert Path(target).is_file(), "场景文件没有写出来"
        assert saved["savedScene"].endswith("scene_layout_check.xml")

        expected = {entry["name"]: entry["pos"] for entry in payload["bodies"]}
        check = mujoco.MjModel.from_xml_path(str(target))
        for name, pos in expected.items():
            body_id = mujoco.mj_name2id(check, mujoco.mjtObj.mjOBJ_BODY, name)
            assert body_id >= 0, "生成场景里找不到 body %s" % name
            for axis, want in zip("xyz", pos):
                got = float(check.body_pos[body_id][{"x": 0, "y": 1, "z": 2}[axis]])
                assert abs(got - want) < 1e-5, "%s.%s 期望 %s 实际 %s" % (
                    name, axis, want, got)
        if payload.get("verifyError"):
            print("注意：%s" % payload["verifyError"], file=sys.stderr)
        print("自检通过：%d 个自由物体，写入 %s" % (len(payload["bodies"]), target.name))
        print(json.dumps(payload, ensure_ascii=False))
    finally:
        import shutil
        shutil.rmtree(tmpdir, ignore_errors=True)
        Path(out_json).with_suffix(".check.json").unlink(missing_ok=True)
    return 0


# ---------------------------------------------------------------------------
# 开窗交互
# ---------------------------------------------------------------------------
def run_viewer(scene_rel, out_json, session_id, log_path):
    import mujoco
    import mujoco.viewer

    scene_path = (WORK4_DIR / scene_rel).resolve()
    scene_text = scene_path.read_text(encoding="utf-8")
    scene_dir = WORK4_DIR / "model" / "scenes"
    target_name = "scene_layout_%s.xml" % datetime.now().strftime("%Y%m%d-%H%M%S")

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    if model.nkey:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    # 机械臂用 home 姿态定住，方便只拖球和盘子
    hold = data.qpos.copy()
    actuator_joints = [int(model.actuator_trnid[i, 0]) for i in range(model.nu)]

    def apply_ctrl():
        for i, jid in enumerate(actuator_joints):
            if jid < 0 or i >= model.nu:
                continue
            if int(model.jnt_type[jid]) == int(mujoco.mjtJoint.mjJNT_HINGE):
                data.ctrl[i] = hold[int(model.jnt_qposadr[jid])]
            else:  # 夹爪：保持张开
                data.ctrl[i] = 0.0

    state = {"save": False}
    saved_count = [0]

    def on_key(key):
        try:
            code = key if isinstance(key, int) else ord(str(key)[0])
        except (TypeError, ValueError):
            return
        if code in (ord("s"), ord("S")):
            state["save"] = True

    if log_path:
        print("启动布置窗口：场景 %s" % scene_rel, file=sys.stderr)

    print("布置窗口已打开。")
    print("  Ctrl + 右键拖动  → 推动选中的物体")
    print("  双击物体         → 选中它，之后可以直接拖动")
    print("  S                → 保存布置")
    print("  关闭窗口         → 也会自动保存")

    with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as viewer:
        viewer.cam.lookat[:] = [0.2, 0.0, 0.98]
        viewer.cam.distance = 2.4
        viewer.cam.azimuth = 130
        viewer.cam.elevation = -22
        start_wall = time.monotonic()
        start_sim = data.time
        while viewer.is_running():
            apply_ctrl()
            target_sim = start_sim + (time.monotonic() - start_wall)
            steps = 0
            while data.time < target_sim and steps < 40:
                mujoco.mj_step(model, data)
                steps += 1
            if steps == 0:
                time.sleep(0.001)
            viewer.sync()
            if state["save"]:
                state["save"] = False
                try:
                    payload, target = save_layout(
                        model, data, mujoco,
                        scene_rel=scene_rel, scene_text=scene_text,
                        scene_dir=scene_dir, out_json=out_json,
                        session_id=session_id, target_name=target_name)
                    saved_count[0] += 1
                    print("已保存：%s" % target)
                    print(json.dumps(payload, ensure_ascii=False))
                except Exception:  # noqa: BLE001
                    traceback.print_exc()

    # 关闭窗口也保存一次
    try:
        payload, target = save_layout(
            model, data, mujoco,
            scene_rel=scene_rel, scene_text=scene_text, scene_dir=scene_dir,
            out_json=out_json, session_id=session_id, target_name=target_name)
        print("窗口已关闭，布置已保存：%s" % target)
        print(json.dumps(payload, ensure_ascii=False))
    except Exception:  # noqa: BLE001
        traceback.print_exc()
        return 1
    _ = saved_count
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description="MuJoCo 交互式布置场景窗口")
    parser.add_argument("--scene", default="model/scene.xml",
                        help="要布置的场景（相对 work4 的路径）")
    parser.add_argument("--out", default=str(RUNNER_DIR / "out" / "layout.json"),
                        help="保存结果的 JSON 路径")
    parser.add_argument("--session", default="", help="会话 id（由 app.py 生成）")
    parser.add_argument("--log", default="", help="日志路径（可选；启动器一般已重定向）")
    parser.add_argument("--check", action="store_true", help="无窗口自检")
    args = parser.parse_args(argv)

    session_id = args.session or "layout-%s" % datetime.now().strftime("%Y%m%d-%H%M%S")
    scene_rel = args.scene.strip().lstrip("./")
    if not (WORK4_DIR / scene_rel).is_file():
        print("找不到场景：%s" % (WORK4_DIR / scene_rel), file=sys.stderr)
        return 2

    if args.check:
        return run_check(scene_rel, args.out, session_id)
    return run_viewer(scene_rel, args.out, session_id, args.log)


if __name__ == "__main__":
    raise SystemExit(main())
