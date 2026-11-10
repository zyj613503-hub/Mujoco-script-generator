#!/usr/bin/env python3
"""MuJoCo 场景 XML 解析（只读、不依赖 MuJoCo）。

给副界面 FIG.02 俯视图提供「当前场景里各物品的真实坐标」：
    GET /api/scene/bodies?scene=model/scene.xml
        -> {ok, scene, include, table, bodies, base, reach}

要点：
  * 用 xml.etree 解析，支持 <include file="..."/>（相对路径按引用它的场景文件目录解析）；
  * 能解析 model/scenes/scene_layout_*.xml（include 写法是 "../ur5e.xml"）；
  * 球半径取 <geom type="sphere" size="...">；盘子 mesh 量不出半径，用常量并标 approximate；
  * 桌子台面 / 腿、机械臂底座都从 XML 里读，不写死演示值。
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

def _find_root(start):
    """向上找含 model/ur5e.xml 的目录（兼容 work4/运行器 与扁平分发包两种布局）。"""
    for parent in [start, *start.parents]:
        if (parent / "model" / "ur5e.xml").is_file():
            return parent
    return start.parent


WORK4_DIR = _find_root(Path(__file__).resolve().parents[1])

DEFAULT_REACH = 0.85                       # 机械臂水平臂展（米）
PLATE_RADIUS = 0.118                       # 盘子外半径（mesh 无法直接量，常量近似）
DEFAULT_BASE = (0.0, 0.0, 0.75)            # ur5e base body 的兜底位置


class SceneError(ValueError):
    """场景文件缺失 / XML 结构错误。"""


# ---------------------------------------------------------------------------
# 基础解析
# ---------------------------------------------------------------------------
def _vec(text, count):
    """把 "0.93 -0.93 0.36" 解析成 [0.93, -0.93, 0.36]，不足补 0。"""
    values = []
    for token in str(text or "").replace(",", " ").split():
        try:
            values.append(float(token))
        except ValueError:
            raise SceneError("坐标里有无法解析的数字：%r" % text)
    while len(values) < count:
        values.append(0.0)
    return values[:count]


def _num(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _within_work4(path):
    try:
        Path(path).resolve().relative_to(WORK4_DIR)
        return True
    except ValueError:
        return False


def _parse(path):
    try:
        return ET.parse(str(path)).getroot()
    except ET.ParseError as exc:
        raise SceneError("场景 XML 解析失败：%s（%s）" % (Path(path).name, exc))
    except OSError as exc:
        raise SceneError("读取场景失败：%s（%s）" % (Path(path).name, exc))


def _load_roots(path):
    """解析主场景 + 递归 <include>，返回 [(root, 所在目录)]，按出现顺序。"""
    roots = []
    seen = set()

    def visit(file_path):
        file_path = Path(file_path).resolve()
        if file_path in seen:
            return
        seen.add(file_path)
        root = _parse(file_path)
        roots.append((root, file_path.parent))
        for inc in root.iter("include"):
            raw = (inc.get("file") or "").strip()
            if not raw:
                continue
            inc_path = (file_path.parent / raw).resolve()
            if not _within_work4(inc_path) or not inc_path.is_file():
                continue
            visit(inc_path)

    visit(path)
    return roots


def _iter_bodies(elem, offset):
    """递归遍历所有 <body>，同时累加出相对 worldbody 的近似世界坐标（忽略旋转）。"""
    for child in list(elem):
        if child.tag == "include":
            continue
        if child.tag == "body":
            local = _vec(child.get("pos"), 3)
            world = [offset[0] + local[0], offset[1] + local[1], offset[2] + local[2]]
            yield child, world
            yield from _iter_bodies(child, world)
        else:
            yield from _iter_bodies(child, offset)


def _has_free_joint(body):
    if body.find("freejoint") is not None:
        return True
    for joint in body.findall("joint"):
        if (joint.get("type") or "").strip().lower() == "free":
            return True
    return False


def _round(values, digits=6):
    return [round(float(v), digits) for v in values]


# ---------------------------------------------------------------------------
# 单个 body
# ---------------------------------------------------------------------------
def _free_body(body, world):
    """自由物体（球 / 盘子）→ 俯视图需要的字段。"""
    name = body.get("name") or ""
    geoms = body.findall("geom")
    kind = None
    radius = None
    mass = None
    approximate = False

    for geom in geoms:
        geom_type = (geom.get("type") or "sphere").strip().lower()
        if geom_type == "sphere" and kind is None:
            kind = "sphere"
            radius = _num(_vec(geom.get("size"), 1)[0], 0.0)
            if geom.get("mass") is not None:
                mass = _num(geom.get("mass"), 0.0)
            break
        if geom_type == "mesh" and kind is None:
            mesh_name = (geom.get("mesh") or "").lower()
            kind = "plate" if ("plate" in mesh_name or "plate" in name.lower()) else "mesh"

    if kind is None:
        return None
    if kind == "plate":
        radius = PLATE_RADIUS
        approximate = True

    item = {
        "name": name,
        "kind": kind,
        "pos": _round(world, 5),
        "radius": round(radius, 6) if radius is not None else None,
    }
    if mass is not None:
        item["mass"] = mass
    if approximate:
        item["approximate"] = True
    return item


def _table(body, world):
    """桌子：台面中心 / 尺寸 / 上表面高度 / 四条腿的 xy。"""
    geoms = body.findall("geom")
    top = None
    for geom in geoms:
        if (geom.get("name") or "") == "table_top":
            top = geom
            break
    if top is None:
        boxes = [g for g in geoms
                 if (g.get("type") or "").strip().lower() == "box"]
        if boxes:
            top = max(boxes, key=lambda g: _vec(g.get("size"), 3)[0]
                      * _vec(g.get("size"), 3)[1])
    if top is None:
        return None

    half = _vec(top.get("size"), 3)
    local = _vec(top.get("pos"), 3)
    legs = []
    for geom in geoms:
        if (geom.get("name") or "").startswith("table_leg"):
            pos = _vec(geom.get("pos"), 3)
            legs.append(_round([world[0] + pos[0], world[1] + pos[1]], 5))

    return {
        "name": body.get("name") or "table",
        "center": _round([world[0] + local[0], world[1] + local[1]], 5),
        "size": _round([2.0 * half[0], 2.0 * half[1]], 5),
        "top_z": round(world[2] + local[2] + half[2], 5),
        "legs": legs,
    }


# ---------------------------------------------------------------------------
# 对外接口
# ---------------------------------------------------------------------------
def parse_scene(scene_rel):
    """解析一个场景（相对 work4 的路径），返回俯视图要用的全部几何信息。"""
    text = str(scene_rel or "").strip()
    path = (WORK4_DIR / text).resolve()
    if not _within_work4(path):
        raise SceneError("场景路径必须位于 work4 内：%s" % text)
    if not path.is_file():
        raise SceneError("场景不存在：%s" % text)

    roots = _load_roots(path)
    main_root = roots[0][0]
    include_raw = ""
    for inc in main_root.iter("include"):
        include_raw = (inc.get("file") or "").strip()
        if include_raw:
            break

    table = None
    base = None
    bodies = []
    for root, _directory in roots:
        for body, world in _iter_bodies(root, [0.0, 0.0, 0.0]):
            name = body.get("name") or ""
            if name == "table" and table is None:
                table = _table(body, world)
            elif name == "base" and base is None:
                base = {"x": round(world[0], 5), "y": round(world[1], 5),
                        "z": round(world[2], 5)}
            if _has_free_joint(body):
                item = _free_body(body, world)
                if item is not None:
                    bodies.append(item)

    if base is None:
        base = {"x": DEFAULT_BASE[0], "y": DEFAULT_BASE[1], "z": DEFAULT_BASE[2]}

    return {
        "ok": True,
        "scene": _rel(path),
        "include": include_raw.rsplit("/", 1)[-1] if include_raw else None,
        "table": table,
        "bodies": bodies,
        "base": base,
        "reach": DEFAULT_REACH,
    }


def body_positions(scene_path):
    """{body name: [x, y, z]}，供保存后的回读校验使用（与俯视图同一套解析）。"""
    path = Path(scene_path).resolve()
    if not path.is_file():
        raise SceneError("场景不存在：%s" % path)
    positions = {}
    for root, _directory in _load_roots(path):
        for body, world in _iter_bodies(root, [0.0, 0.0, 0.0]):
            name = body.get("name")
            if name and name not in positions:
                positions[name] = _round(world, 6)
    return positions


def _rel(path):
    try:
        return Path(path).resolve().relative_to(WORK4_DIR).as_posix()
    except ValueError:
        return Path(path).as_posix()


if __name__ == "__main__":  # 手工自检：python server/scene.py [场景相对路径]
    import json
    import sys

    target = sys.argv[1] if len(sys.argv) > 1 else "model/scene.xml"
    print(json.dumps(parse_scene(target), ensure_ascii=False, indent=2))
