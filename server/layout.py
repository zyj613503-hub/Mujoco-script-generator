"""运行器脚本生成 / 解析的纯逻辑（不依赖 MuJoCo，便于单独测试）。

职责：
  * waypoints（网页里的路点数组）→ action/arm_motion.py 参数块的文本替换；
  * 生成脚本的落盘与 compile() 语法校验；
  * 读回已有脚本的 WAYPOINTS，估算时长，供 /api/recent 使用。

所有路径都相对项目根 work4，形如 "model/scene.xml"。
"""

from __future__ import annotations

import ast
import os
import re
from datetime import datetime
from pathlib import Path

# 与 action/arm_motion.py 里「② 参数区」的默认值保持一致
DEFAULT_APPROACH = 5.0     # 初始姿态 → 第一个路点
DEFAULT_SEGMENT = 6.0      # 相邻路点之间
DEFAULT_TAIL_HOLD = 1.0    # 走完路点后再停多久

NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,64}$")
GRIPPER_NAMES = ("open", "close")


# ---------------------------------------------------------------------------
# 路点：清洗 / 格式化 / 统计
# ---------------------------------------------------------------------------
def normalize_waypoints(raw):
    """把前端传来的路点统一成 [{x,y,z,hold,gripper}]。

    同时接受 dict 与数组两种写法：
        {"x":0.5,"y":-0.05,"z":1.1,"hold":0.5,"gripper":"open"}
        [0.5, -0.05, 1.1, 0.5, "open"]
    """
    if raw is None:
        raise ValueError("waypoints 不能为空")
    if not isinstance(raw, list):
        raise ValueError("waypoints 必须是数组")
    out = []
    for i, wp in enumerate(raw):
        if isinstance(wp, dict):
            def get(key, default=None, _wp=wp):
                return _wp.get(key, default)
        elif isinstance(wp, (list, tuple)):
            if len(wp) < 3:
                raise ValueError("第 %d 个路点缺少坐标" % (i + 1))
            keys = {"x": 0, "y": 1, "z": 2, "hold": 3, "gripper": 4}

            def get(key, default=None, _wp=wp, _keys=keys):
                idx = _keys[key]
                if idx < len(_wp):
                    return _wp[idx]
                return default
        else:
            raise ValueError("第 %d 个路点格式无法识别" % (i + 1))

        def number(key, default):
            value = get(key, default)
            try:
                return float(value)
            except (TypeError, ValueError):
                raise ValueError("第 %d 个路点的 %s 不是数字：%r"
                                 % (i + 1, key, value))

        x, y, z = number("x", 0.0), number("y", 0.0), number("z", 0.0)
        hold = number("hold", 0.0)
        if hold < 0:
            raise ValueError("第 %d 个路点的 hold 不能是负数" % (i + 1))
        grip = get("gripper", None)
        if isinstance(grip, str):
            grip = grip.strip().lower()
            grip = grip if grip in GRIPPER_NAMES else None
        elif isinstance(grip, (int, float)) and not isinstance(grip, bool):
            grip = float(grip)
        else:
            grip = None
        out.append({"x": x, "y": y, "z": z, "hold": hold, "gripper": grip})
    return out


def _hold_of(wp):
    try:
        if isinstance(wp, dict):
            return float(wp.get("hold", 0.0) or 0.0)
        return float(wp[3])
    except (IndexError, TypeError, ValueError):
        return 0.0


def duration_seconds(waypoints,
                     approach=DEFAULT_APPROACH,
                     segment=DEFAULT_SEGMENT,
                     tail=DEFAULT_TAIL_HOLD):
    """与页面估算一致的总时长（秒）：接近 5s + 分段 (n-1)×6s + 各点停留 + 收尾 1s。"""
    n = len(waypoints)
    holds = sum(_hold_of(wp) for wp in waypoints)
    return round(approach + max(n - 1, 0) * segment + holds + tail, 2)


def gripper_changes(waypoints):
    """夹爪状态发生变化的次数（None = 保持不动，不参与计数）。"""
    changes, prev = 0, None
    for wp in waypoints:
        if isinstance(wp, dict):
            grip = wp.get("gripper")
        else:
            grip = wp[4] if len(wp) > 4 else None
        if grip is None:
            continue
        if prev is not None and grip != prev:
            changes += 1
        prev = grip
    return changes


def format_waypoints_block(waypoints):
    """生成与模板风格一致的 WAYPOINTS 参数块（带中文行注释）。"""
    lines = [
        "WAYPOINTS = [",
        "    # 由 MuJoCo 运行器生成：每行 [x, y, z, 停留秒数, 夹爪状态]",
    ]
    for i, wp in enumerate(waypoints, 1):
        grip = wp["gripper"]
        if grip is None:
            grip_text = "None"
        elif isinstance(grip, str):
            grip_text = '"%s"' % grip
        else:
            grip_text = "%g" % grip
        lines.append(
            '    [%.3f, %.3f, %.3f, %4.2f, %s],  # 路点 %d'
            % (wp["x"], wp["y"], wp["z"], wp["hold"], grip_text, i)
        )
    lines.append("]")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# 参数块替换
# ---------------------------------------------------------------------------
def _safe_rel(path, what):
    text = str(path or "").strip().replace("\\", "/")
    if not text:
        raise ValueError("%s 为空" % what)
    if text.startswith("/") or re.match(r"^[A-Za-z]:", text):
        raise ValueError("%s 必须是相对 work4 的路径：%s" % (what, text))
    parts = [p for p in text.split("/") if p not in ("", ".")]
    if any(p == ".." for p in parts):
        raise ValueError("%s 不允许包含 ..：%s" % (what, text))
    return "/".join(parts)


def _set_number(text, param, value):
    """替换 PARAM = 数字，保留行尾中文注释。"""
    pattern = re.compile(r"^(%s\s*=\s*)([0-9]+(?:\.[0-9]+)?)" % re.escape(param), re.M)
    new_text, count = pattern.subn(
        lambda m: m.group(1) + ("%.1f" % value), text, count=1)
    return new_text, count == 1


def _set_string(text, param, value):
    """替换 PARAM = "..."，保留行尾中文注释。"""
    pattern = re.compile(r'^(%s\s*=\s*)"[^"]*"' % re.escape(param), re.M)
    new_text, count = pattern.subn(
        lambda m: m.group(1) + '"%s"' % value, text, count=1)
    return new_text, count == 1


def build_script(template, waypoints, *,
                 model="model/ur5e.xml",
                 scene="model/scene.xml",
                 approach=DEFAULT_APPROACH,
                 segment=DEFAULT_SEGMENT,
                 now=None,
                 extra_comments=None):
    """在模板文本上替换参数块，返回 (脚本文本, warnings, notes)。

    * WAYPOINTS 整块替换；
    * APPROACH_TIME / SEGMENT_TIME 数值替换；
    * XML_PATH / IK_XML_PATH 路径替换；
    * ROOT 改成「从脚本所在目录往上找 model/ur5e.xml」的写法：脚本可能落在
      运行器/scripts/，也可能落在分发包的 scripts/，不能写死层级；
    * extra_comments：生成时间那一行之后追加的整行注释（记录场景布置用）。
    没匹配上的项不报错，只记进 warnings。
    """
    warnings, notes = [], []
    text = template

    block = format_waypoints_block(waypoints)
    new_text, count = re.subn(
        r"^WAYPOINTS\s*=\s*\[.*?^\]", lambda m: block, text, flags=re.S | re.M)
    if count:
        text = new_text
    else:
        warnings.append("未找到 WAYPOINTS 参数块，已跳过替换")

    for param, value in (("APPROACH_TIME", approach), ("SEGMENT_TIME", segment)):
        text, ok = _set_number(text, param, value)
        if not ok:
            warnings.append("未找到 %s，已跳过替换" % param)

    text, ok = _set_string(text, "XML_PATH", _safe_rel(scene, "scene"))
    if not ok:
        warnings.append("未找到 XML_PATH，已跳过替换")
    text, ok = _set_string(text, "IK_XML_PATH", _safe_rel(model, "model"))
    if not ok:
        warnings.append("未找到 IK_XML_PATH，已跳过替换")

    # 模板里 ROOT 已经是「向上找 model/ur5e.xml」的写法时不用动；
    # 老模板（ROOT = ...parent.parent）才替换，免得在分发包里层级对不上。
    if 'p / "model" / "ur5e.xml"' in text:
        notes.append("ROOT 已是自动查找写法")
    else:
        root_block = (
            "# 脚本可能被放在 运行器/scripts/（开发目录）或分发包的 scripts/：\n"
            "# 从脚本所在目录往上找含 model/ur5e.xml 的那一层当项目根。\n"
            "ROOT = next((p for p in Path(__file__).resolve().parents\n"
            "             if (p / \"model\" / \"ur5e.xml\").is_file()),\n"
            "            Path(__file__).resolve().parent.parent)")
        root_pattern = re.compile(
            r"^ROOT\s*=\s*Path\(__file__\)\.resolve\(\)\.parent\.parent\s*$",
            re.M)
        text, count = root_pattern.subn(lambda _m: root_block, text, count=1)
        if not count:
            warnings.append("未找到 ROOT 定义，脚本里的相对路径可能解析错误")
        else:
            notes.append("ROOT 已改为「向上查找 model/ur5e.xml」的写法")

    stamp = (now or datetime.now()).strftime("%Y-%m-%d %H:%M:%S")
    header = "# 由 MuJoCo 运行器生成 · %s\n" % stamp
    for line in extra_comments or []:
        line = str(line).rstrip("\n")
        if line:
            header += line + "\n"
    return header + text, warnings, notes


def write_script(scripts_dir, name, text):
    """先写临时文件 → compile() 校验 → 原子改名。返回最终路径。"""
    scripts_dir = Path(scripts_dir)
    scripts_dir.mkdir(parents=True, exist_ok=True)
    final = scripts_dir / ("%s.py" % name)
    tmp = scripts_dir / (".%s.py.tmp" % name)
    tmp.write_text(text, encoding="utf-8")
    try:
        compile(text, str(final), "exec")
    except SyntaxError as exc:
        tmp.unlink(missing_ok=True)
        raise ValueError("生成的脚本语法错误：第 %s 行 %s" % (exc.lineno, exc.msg))
    os.replace(tmp, final)
    return final


def generate(scripts_dir, name, waypoints, *, template_text, model, scene,
             now=None, extra_comments=None):
    """生成一个脚本文件，返回 {path, duration, waypoints, warnings, notes}。"""
    if not NAME_RE.match(name or ""):
        raise ValueError("脚本名只能包含字母、数字、下划线、短横线（1-64 位）")
    wps = normalize_waypoints(waypoints)
    text, warnings, notes = build_script(
        template_text, wps, model=model, scene=scene, now=now,
        extra_comments=extra_comments)
    path = write_script(scripts_dir, name, text)
    return {
        "path": "scripts/%s.py" % name,
        "duration": duration_seconds(wps),
        "waypoints": len(wps),
        "warnings": warnings,
        "notes": notes,
        "abs_path": str(path),
    }


# ---------------------------------------------------------------------------
# 读回脚本
# ---------------------------------------------------------------------------
def parse_waypoints_source(text):
    """从脚本源码里静态取出 WAYPOINTS（用 ast，不执行脚本）。取不到返回 None。"""
    try:
        tree = ast.parse(text)
    except SyntaxError:
        return None
    for node in tree.body:
        if isinstance(node, ast.Assign):
            targets = [t.id for t in node.targets if isinstance(t, ast.Name)]
            if "WAYPOINTS" in targets:
                try:
                    return ast.literal_eval(node.value)
                except (ValueError, SyntaxError):
                    return None
    return None


def read_script_meta(path, out_dir):
    """解析一个脚本文件 → recent 列表条目所需的信息。"""
    path = Path(path)
    text = path.read_text(encoding="utf-8", errors="replace")
    wps = parse_waypoints_source(text)
    wps = wps if isinstance(wps, list) else []
    ran = (Path(out_dir) / ("%s.csv" % path.stem)).exists() or "# ran" in text
    mtime = path.stat().st_mtime
    return {
        "file": "scripts/%s.py" % path.stem,
        "name": path.stem,
        "waypoints": len(wps),
        "duration": duration_seconds(wps),
        "opened_at": datetime.fromtimestamp(mtime).strftime("%Y-%m-%d %H:%M"),
        "mtime": round(mtime, 3),
        "status": "已运行" if ran else "草稿",
    }


def scan_recent(scripts_dir, out_dir, limit=20):
    """扫 运行器/scripts/*.py，按修改时间倒序返回最近脚本。"""
    scripts_dir = Path(scripts_dir)
    if not scripts_dir.is_dir():
        return []
    files = [p for p in scripts_dir.glob("*.py") if not p.name.startswith(".")]
    files.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    items = []
    for p in files[:limit]:
        try:
            items.append(read_script_meta(p, out_dir))
        except OSError:
            continue
    return items

# ---------------------------------------------------------------------------
# 读取单个脚本（/api/script/load）
# ---------------------------------------------------------------------------
def _assigned_literals(tree):
    """模块顶层 NAME = 字面量 的赋值表（ast.literal_eval，不执行脚本）。"""
    values = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if not isinstance(target, ast.Name):
                continue
            try:
                values[target.id] = ast.literal_eval(node.value)
            except (ValueError, SyntaxError, TypeError):
                continue
    return values


def _number_or(value, default):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return default
    return float(value)


def load_script(path):
    """静态解析一个生成的脚本，返回 /api/script/load 需要的字段。

    WAYPOINTS / XML_PATH / IK_XML_PATH / APPROACH_TIME / SEGMENT_TIME 都从
    字面量赋值里取（ast），绝不 exec 用户脚本。夹爪写数字时按脚本里的
    GRIPPER_OPEN / GRIPPER_CLOSE 近似映射回 "open" / "close"，认不出给 None。
    """
    path = Path(path)
    text = path.read_text(encoding="utf-8", errors="replace")
    try:
        tree = ast.parse(text)
    except SyntaxError as exc:
        raise ValueError("脚本语法错误：第 %s 行 %s" % (exc.lineno, exc.msg))

    values = _assigned_literals(tree)
    raw_waypoints = values.get("WAYPOINTS")
    if not isinstance(raw_waypoints, list):
        raw_waypoints = []

    open_value = _number_or(values.get("GRIPPER_OPEN"), 0.0)
    close_value = _number_or(values.get("GRIPPER_CLOSE"), 0.005)

    waypoints = []
    for wp in raw_waypoints:
        if not isinstance(wp, (list, tuple)) or len(wp) < 3:
            continue

        def at(index, default):
            try:
                return float(wp[index])
            except (IndexError, TypeError, ValueError):
                return default

        grip = wp[4] if len(wp) > 4 else None
        if isinstance(grip, str):
            grip = grip.strip().lower()
            grip = grip if grip in GRIPPER_NAMES else None
        elif isinstance(grip, (int, float)) and not isinstance(grip, bool):
            value = float(grip)
            if abs(value - close_value) < 1e-9:
                grip = "close"
            elif abs(value - open_value) < 1e-9:
                grip = "open"
            else:
                grip = None
        else:
            grip = None

        waypoints.append({
            "x": at(0, 0.0),
            "y": at(1, 0.0),
            "z": at(2, 0.0),
            "hold": at(3, 0.0),
            "gripper": grip,
        })

    mtime = path.stat().st_mtime
    scene = values.get("XML_PATH")
    model = values.get("IK_XML_PATH")
    return {
        "file": "scripts/%s" % path.name,
        "name": path.stem,
        "scene": scene if isinstance(scene, str) else None,
        "model": model if isinstance(model, str) else None,
        "waypoints": waypoints,
        "approach_time": _number_or(values.get("APPROACH_TIME"), DEFAULT_APPROACH),
        "segment_time": _number_or(values.get("SEGMENT_TIME"), DEFAULT_SEGMENT),
        "mtime": round(mtime, 3),
    }
