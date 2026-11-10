#!/usr/bin/env python3
"""MuJoCo 运行器 · 本地服务（静态站点 + /api/*）。

只用标准库：ThreadingHTTPServer 监听 127.0.0.1，默认端口 8770（RUNNER_PORT 可覆盖）。
静态根 = 运行器/web；其余 /api/ 走 JSON 接口。

启动：
    python server/app.py（或直接双击 start.bat）
    浏览器打开 http://127.0.0.1:8770/

目录约定（相对项目根）：
    action/arm_motion.py    脚本模板（只读）
    model/*.xml             机器人模型 + 默认场景
    model/scenes/*.xml      布置窗口存下来的场景
    scripts/*.py            生成的脚本
    out/*.json|log          布置会话的保存结果与日志
"""

from __future__ import annotations

import json
import mimetypes
import os
import re
import subprocess
import sys
import threading
import time
import traceback
import uuid
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlsplit

# server/ 目录加入 sys.path，直接 python server/app.py 也能 import 同级模块
SERVER_DIR = Path(__file__).resolve().parent
if str(SERVER_DIR) not in sys.path:
    sys.path.insert(0, str(SERVER_DIR))

import layout as layoutlib  # noqa: E402  （同目录模块）
import scene as scenelib  # noqa: E402  （场景 XML 解析）

RUNNER_DIR = SERVER_DIR.parent          # 运行器（分发包里就是包根）


def _find_root(start):
    """向上找含 model/ur5e.xml 的目录，兼容两种布局：

        work4/运行器/server/app.py   → 项目根是 work4
        运行器打包/server/app.py      → 项目根就是 运行器打包
    """
    for parent in [start, *start.parents]:
        if (parent / "model" / "ur5e.xml").is_file():
            return parent
    return start.parent


WORK4_DIR = _find_root(RUNNER_DIR)      # 项目根：模型、脚本模板、虚拟环境都在这下面
WEB_DIR = (RUNNER_DIR / "web").resolve()
SCRIPTS_DIR = RUNNER_DIR / "scripts"
OUT_DIR = RUNNER_DIR / "out"
TEMPLATE_PATH = WORK4_DIR / "action" / "arm_motion.py"
VENV_BIN = WORK4_DIR / ".venv" / ("Scripts" if os.name == "nt" else "bin")
PYTHON_EXE = VENV_BIN / ("python.exe" if os.name == "nt" else "python")
if not PYTHON_EXE.is_file():
    PYTHON_EXE = Path(sys.executable)
MJPYTHON = VENV_BIN / "mjpython"               # 只有 macOS 用得到
if not MJPYTHON.is_file():
    MJPYTHON = Path(sys.executable).parent / "mjpython"
MODEL_DIR = WORK4_DIR / "model"
SCENES_DIR = MODEL_DIR / "scenes"
LAYOUT_JSON = OUT_DIR / "layout.json"
CURRENT_LAYOUT_JSON = OUT_DIR / "current_layout.json"

HOST = "127.0.0.1"
PORT = int(os.environ.get("RUNNER_PORT", "8770"))
MAX_BODY = 5 * 1024 * 1024

MIME_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".htm": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".mjs": "application/javascript; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".svg": "image/svg+xml",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".gif": "image/gif",
    ".webp": "image/webp",
    ".ico": "image/x-icon",
    ".woff": "font/woff",
    ".woff2": "font/woff2",
    ".ttf": "font/ttf",
    ".txt": "text/plain; charset=utf-8",
    ".md": "text/markdown; charset=utf-8",
    ".map": "application/json; charset=utf-8",
}

_HEALTH = None
_HEALTH_LOCK = threading.Lock()

_SESSIONS = {}
_SESSIONS_LOCK = threading.Lock()


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def _json_response(handler, payload, status=200):
    body = json.dumps(payload, ensure_ascii=False, indent=None).encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(body)))
    _no_cache(handler)
    handler.end_headers()
    handler.wfile.write(body)


def _no_cache(handler):
    # 允许 file:// 直接双击打开页面时也能访问本地接口（origin 为 null）
    handler.send_header("Access-Control-Allow-Origin", "*")
    handler.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
    handler.send_header("Access-Control-Allow-Headers", "Content-Type")
    handler.send_header("Access-Control-Max-Age", "600")
    handler.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
    handler.send_header("Pragma", "no-cache")
    handler.send_header("Expires", "0")
    handler.send_header("X-Content-Type-Options", "nosniff")


def _rel(path):
    """绝对路径 → 相对 work4 的 posix 字符串。"""
    try:
        return Path(path).resolve().relative_to(WORK4_DIR).as_posix()
    except ValueError:
        return Path(path).as_posix()


def _exists_rel(rel_path):
    text = str(rel_path or "").strip()
    if not text:
        return None
    candidate = (WORK4_DIR / text).resolve()
    try:
        candidate.relative_to(WORK4_DIR)
    except ValueError:
        return None
    return candidate if candidate.is_file() else None


def _scan_xml(directory, prefix):
    if not directory.is_dir():
        return []
    names = sorted(p.name for p in directory.glob("*.xml") if p.is_file())
    return ["%s/%s" % (prefix, name) for name in names]


def _health_info():
    global _HEALTH
    with _HEALTH_LOCK:
        if _HEALTH is not None:
            return _HEALTH
        info = {"ok": True, "mujoco": "unknown", "mink": "unknown"}
        try:
            import mujoco
            info["mujoco"] = getattr(mujoco, "__version__", "unknown")
        except Exception as exc:  # noqa: BLE001
            info["ok"] = False
            info["mujoco"] = "unavailable"
            info["mujoco_error"] = str(exc)
        try:
            import importlib.metadata as metadata
            info["mink"] = metadata.version("mink")
        except Exception:  # noqa: BLE001
            try:
                import mink  # noqa: F401
                info["mink"] = "installed"
            except Exception as exc:  # noqa: BLE001
                info["ok"] = False
                info["mink"] = "unavailable"
                info["mink_error"] = str(exc)
        _HEALTH = info
        return info


# ---------------------------------------------------------------------------
# /api/* 实现
# ---------------------------------------------------------------------------
def api_health(_query, _body):
    return _health_info()


def api_models(_query, _body):
    models = _scan_xml(MODEL_DIR, "model")
    scenes = _scan_xml(SCENES_DIR, "model/scenes")
    model = "model/ur5e.xml" if "model/ur5e.xml" in models else (
        models[0] if models else "model/ur5e.xml")
    scene = "model/scene.xml" if "model/scene.xml" in scenes + models else (
        scenes[0] if scenes else "model/scene.xml")
    return {"model": model, "scene": scene, "models": models, "scenes": scenes}


def api_files(query, _body):
    kind = (query.get("kind", ["model"])[0] or "model").strip().lower()
    if kind == "scene":
        items = _scan_xml(SCENES_DIR, "model/scenes")
    elif kind == "model":
        items = _scan_xml(MODEL_DIR, "model")
    elif kind == "script":
        items = []
        if SCRIPTS_DIR.is_dir():
            scripts = [p for p in SCRIPTS_DIR.glob("*.py")
                       if p.is_file() and not p.name.startswith(".")]
            scripts.sort(key=lambda p: p.stat().st_mtime, reverse=True)
            items = ["scripts/%s" % p.name for p in scripts]
    else:
        return {"ok": False,
                "error": "kind 只能是 model、scene 或 script", "items": []}
    return {"items": items}


def api_scene_bodies(query, _body):
    """场景里桌子 / 球 / 盘子的真实几何（副界面 FIG.02 俯视图用）。"""
    scene = (query.get("scene", ["model/scene.xml"])[0] or "model/scene.xml").strip()
    try:
        return scenelib.parse_scene(scene)
    except scenelib.SceneError as exc:
        return {"ok": False, "error": str(exc)}


def api_layout_current(_query, _body):
    """最近一次布置（out/current_layout.json）。"""
    try:
        payload = json.loads(CURRENT_LAYOUT_JSON.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {"ok": True, "has": False}
    if not isinstance(payload, dict):
        return {"ok": True, "has": False}
    return {
        "ok": True,
        "has": True,
        "savedAt": payload.get("savedAt"),
        "scene": payload.get("scene"),
        "savedScene": payload.get("savedScene"),
        "bodies": payload.get("bodies", []),
    }


def _script_path(raw):
    """把 "scripts/x.py" / "x.py" 收敛成 scripts/ 下的绝对路径，非法返回 None。"""
    text = str(raw or "").strip().replace("\\", "/")
    name = text.rsplit("/", 1)[-1]
    if not name.endswith(".py"):
        return None
    if not layoutlib.NAME_RE.match(name[:-3]):
        return None
    path = (SCRIPTS_DIR / name).resolve()
    try:
        path.relative_to(SCRIPTS_DIR.resolve())
    except ValueError:
        return None
    return path


def api_script_load(query, _body):
    """静态解析已有脚本（ast，不执行）：路点 + 场景/模型路径 + 时间参数。"""
    raw = (query.get("file", [""])[0] or "").strip()
    if not raw:
        return {"ok": False, "error": "缺少 file 参数"}
    path = _script_path(raw)
    if path is None:
        return {"ok": False, "error": "脚本路径不合法：%s" % raw}
    if not path.is_file():
        return {"ok": False, "error": "脚本不存在：%s" % raw}
    try:
        payload = layoutlib.load_script(path)
    except (OSError, ValueError) as exc:
        return {"ok": False, "error": str(exc)}
    return {"ok": True, **payload}


def api_recent(_query, _body):
    return {"items": layoutlib.scan_recent(SCRIPTS_DIR, OUT_DIR, limit=20)}


def _preview_stats(waypoints):
    base = (0.0, 0.0, 0.75)
    radius = 0.85
    # 下限取桌面高度附近（桌面 z=0.75、球心 0.772，手指抱球腰时爪尖会略低于台面）
    z_min, z_max = 0.74, 1.55
    unreachable = []
    worst_mm = 0.0
    for i, wp in enumerate(waypoints):
        horizontal = ((wp["x"] - base[0]) ** 2 + (wp["y"] - base[1]) ** 2) ** 0.5
        if horizontal > radius:
            overshoot = (horizontal - radius) * 1000.0
            worst_mm = max(worst_mm, overshoot)
            unreachable.append({
                "index": i,
                "reason": "水平距离 %.3f m 超出臂展 %.2f m（差 %.0f mm）"
                          % (horizontal, radius, overshoot),
            })
        if wp["z"] < z_min:
            worst_mm = max(worst_mm, (z_min - wp["z"]) * 1000.0)
            unreachable.append({
                "index": i,
                "reason": "z = %.3f m 低于可抓取下限 %.2f m" % (wp["z"], z_min),
            })
        elif wp["z"] > z_max:
            worst_mm = max(worst_mm, (wp["z"] - z_max) * 1000.0)
            unreachable.append({
                "index": i,
                "reason": "z = %.3f m 高于可抓取上限 %.2f m" % (wp["z"], z_max),
            })

    # 关节速度粗估：相邻路点距离 / 每段 6s，再按 0.35 m 等效连杆长度换成 rad/s
    peak = 0.0
    for a, b in zip(waypoints, waypoints[1:]):
        dist = ((a["x"] - b["x"]) ** 2 + (a["y"] - b["y"]) ** 2
                + (a["z"] - b["z"]) ** 2) ** 0.5
        peak = max(peak, dist / layoutlib.DEFAULT_SEGMENT / 0.35)
    if not peak and waypoints:
        peak = 0.35
    return {
        "duration": layoutlib.duration_seconds(waypoints),
        "gripper_changes": layoutlib.gripper_changes(waypoints),
        "reachable": not unreachable,
        "unreachable": unreachable,
        "max_ik_mm": round(worst_mm, 1),
        "joint_speed": round(peak, 2),
        "estimated": True,
    }


def api_preview(_query, body):
    try:
        waypoints = layoutlib.normalize_waypoints(body.get("waypoints"))
    except ValueError as exc:
        return {"ok": False, "error": str(exc)}
    stats = _preview_stats(waypoints)
    return {"ok": True, **stats}


def _layout_comments(scene):
    """若当前布置就是这次要生成的场景，返回记录布置的两行注释。"""
    try:
        payload = json.loads(CURRENT_LAYOUT_JSON.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return []
    if not isinstance(payload, dict):
        return []
    if payload.get("savedScene") != scene:
        return []
    parts = []
    for entry in payload.get("bodies") or []:
        pos = entry.get("pos") or []
        if len(pos) >= 3:
            parts.append("%s (%.4f, %.4f, %.4f)"
                         % (entry.get("name", "body"), pos[0], pos[1], pos[2]))
    if not parts:
        return []
    saved_at = str(payload.get("savedAt") or "").replace("T", " ")
    return [
        "# 场景布置：" + " · ".join(parts),
        "# 布置保存于 %s（out/layout.json）" % saved_at,
    ]


def api_generate(_query, body):
    name = str(body.get("name") or "").strip()
    try:
        waypoints = layoutlib.normalize_waypoints(body.get("waypoints"))
    except ValueError as exc:
        return {"ok": False, "error": str(exc)}
    if not layoutlib.NAME_RE.match(name):
        return {"ok": False,
                "error": "脚本名只能包含字母、数字、下划线、短横线（1-64 位）"}
    if not TEMPLATE_PATH.is_file():
        return {"ok": False, "error": "找不到模板 %s" % TEMPLATE_PATH}
    model = str(body.get("model") or "model/ur5e.xml").strip()
    scene = str(body.get("scene") or "model/scene.xml").strip()
    if _exists_rel(scene) is None:
        return {"ok": False, "error": "场景不存在：%s" % scene}
    if _exists_rel(model) is None:
        return {"ok": False, "error": "模型不存在：%s" % model}
    annotations = _layout_comments(scene)
    try:
        result = layoutlib.generate(
            SCRIPTS_DIR, name, waypoints,
            template_text=TEMPLATE_PATH.read_text(encoding="utf-8"),
            model=model, scene=scene, extra_comments=annotations)
    except ValueError as exc:
        return {"ok": False, "error": str(exc)}
    except OSError as exc:
        return {"ok": False, "error": "写入脚本失败：%s" % exc}
    return {
        "ok": True,
        "path": result["path"],
        "scene": scene,
        "duration": result["duration"],
        "waypoints": result["waypoints"],
        "warnings": result["warnings"],
        "notes": result["notes"],
        "layoutRecorded": bool(annotations),
    }


def _tail(path, lines=60):
    try:
        text = Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""
    return "\n".join(text.splitlines()[-lines:])


def _new_session_id():
    return "layout-%s-%s" % (datetime.now().strftime("%Y%m%d-%H%M%S"),
                             uuid.uuid4().hex[:4])


def api_layout_launch(_query, body):
    scene = str(body.get("scene") or "model/scene.xml").strip()
    model = str(body.get("model") or "model/ur5e.xml").strip()
    if _exists_rel(scene) is None:
        return {"ok": False, "error": "场景不存在：%s" % scene}
    if _exists_rel(model) is None:
        return {"ok": False, "error": "模型不存在：%s" % model}

    with _SESSIONS_LOCK:
        for sid, info in _SESSIONS.items():
            if info["proc"].poll() is None:
                return {"ok": True, "sessionId": sid,
                        "message": "布置窗口已在运行（会话 %s），请先在窗口里操作" % sid}

        OUT_DIR.mkdir(parents=True, exist_ok=True)
        session_id = _new_session_id()
        log_path = OUT_DIR / ("layout-%s.log" % datetime.now().strftime("%Y%m%d-%H%M%S"))
        editor = SERVER_DIR / "layout_editor.py"
        # macOS 开窗必须走 mjpython 包装器；Windows / Linux 直接用 python
        python = PYTHON_EXE
        use_mjpython = sys.platform == "darwin" and MJPYTHON.is_file()
        if use_mjpython:
            cmd = [str(python), str(MJPYTHON)]
        else:
            cmd = [str(python)]
        cmd += [str(editor), "--scene", scene, "--out", str(LAYOUT_JSON),
                "--session", session_id, "--log", str(log_path)]

        env = dict(os.environ)
        env["PYTHONUNBUFFERED"] = "1"
        env.setdefault("MJ_LOG", str(log_path))
        try:
            log_file = open(log_path, "w", encoding="utf-8")
        except OSError as exc:
            return {"ok": False, "error": "无法写日志 %s：%s" % (log_path, exc)}
        try:
            proc = subprocess.Popen(
                cmd, cwd=str(WORK4_DIR), stdout=log_file, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, env=env,
                start_new_session=(os.name != "nt"))
        except OSError as exc:
            log_file.close()
            return {"ok": False, "error": "启动布置窗口失败：%s" % exc}
        _SESSIONS[session_id] = {
            "proc": proc,
            "log": log_path,
            "scene": scene,
            "model": model,
            "started": time.time(),
            "log_file": log_file,
        }
        message = "布置窗口已启动。拖动球/盘子摆好位置后按 S 保存；直接关窗口也会保存。"
        if sys.platform == "darwin" and not use_mjpython:
            message += "（未找到 mjpython，已退回普通 python，可能无法开窗）"
        return {"ok": True, "sessionId": session_id, "message": message}


def _read_layout_json(session_id):
    try:
        with open(LAYOUT_JSON, "r", encoding="utf-8") as fh:
            payload = json.load(fh)
    except (OSError, ValueError):
        return None
    if payload.get("sessionId") != session_id:
        return None
    return payload


def api_layout_status(query, _body):
    session_id = (query.get("sessionId", [""])[0] or "").strip()
    if not session_id:
        return {"ok": False, "state": "error", "error": "缺少 sessionId"}
    payload = _read_layout_json(session_id)
    with _SESSIONS_LOCK:
        info = _SESSIONS.get(session_id)
    if info is None:
        if payload:
            return {"ok": True, "state": "saved",
                    "savedScene": payload.get("savedScene"),
                    "bodies": payload.get("bodies", []),
                    "savedAt": payload.get("savedAt")}
        return {"ok": False, "state": "error",
                "error": "未知会话：%s（服务可能重启过）" % session_id}

    log_path = info["log"]
    if payload:
        return {"ok": True, "state": "saved",
                "savedScene": payload.get("savedScene"),
                "bodies": payload.get("bodies", []),
                "savedAt": payload.get("savedAt"),
                "log": _tail(log_path, 12)}

    code = info["proc"].poll()
    if code is None:
        return {"ok": True, "state": "running", "log": _tail(log_path, 12)}
    payload = _read_layout_json(session_id)
    if payload:
        return {"ok": True, "state": "saved",
                "savedScene": payload.get("savedScene"),
                "bodies": payload.get("bodies", []),
                "savedAt": payload.get("savedAt")}
    if code == 0:
        return {"ok": True, "state": "closed", "log": _tail(log_path, 60)}
    return {"ok": False, "state": "error", "exitCode": code,
            "log": _tail(log_path, 60)}


ROUTES = {
    "/api/health": ("GET", api_health),
    "/api/models": ("GET", api_models),
    "/api/files": ("GET", api_files),
    "/api/recent": ("GET", api_recent),
    "/api/scene/bodies": ("GET", api_scene_bodies),
    "/api/layout/current": ("GET", api_layout_current),
    "/api/script/load": ("GET", api_script_load),
    "/api/preview": ("POST", api_preview),
    "/api/generate": ("POST", api_generate),
    "/api/layout/launch": ("POST", api_layout_launch),
    "/api/layout/status": ("GET", api_layout_status),
}


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------
class RunnerHandler(BaseHTTPRequestHandler):
    server_version = "MuJoCoRunner/0.1"
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):  # 安静一点，只留一行访问日志
        sys.stderr.write("[%s] %s\n" % (datetime.now().strftime("%H:%M:%S"),
                                        fmt % args))

    # -- 请求分发 --------------------------------------------------------
    def do_GET(self):
        self._handle("GET")

    def do_POST(self):
        self._handle("POST")

    def do_HEAD(self):
        self._handle("HEAD")

    def do_OPTIONS(self):  # CORS 预检（file:// 直接打开页面时需要）
        self.send_response(204)
        _no_cache(self)
        self.send_header("Content-Length", "0")
        self.end_headers()

    def _handle(self, method):
        try:
            split = urlsplit(self.path)
            path = unquote(split.path)
            if path.startswith("/api/"):
                self._handle_api(method, path.rstrip("/"), parse_qs(split.query))
            else:
                self._handle_static(method, path)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception:  # noqa: BLE001
            traceback.print_exc()
            try:
                _json_response(self, {"ok": False, "error": "服务内部错误"}, 500)
            except Exception:  # noqa: BLE001
                pass

    def _handle_api(self, method, path, query):
        route = ROUTES.get(path)
        if route is None:
            _json_response(self, {"ok": False, "error": "未知接口：%s" % path}, 404)
            return
        allowed, fn = route
        if method == "HEAD":
            method = "GET"
        if method != allowed:
            _json_response(self, {"ok": False,
                                  "error": "该接口只支持 %s" % allowed}, 405)
            return
        body = {}
        if method == "POST":
            length = int(self.headers.get("Content-Length") or 0)
            if length > MAX_BODY:
                _json_response(self, {"ok": False, "error": "请求体过大"}, 413)
                return
            raw = self.rfile.read(length) if length else b""
            if raw:
                try:
                    body = json.loads(raw.decode("utf-8"))
                except (UnicodeDecodeError, ValueError):
                    _json_response(self, {"ok": False, "error": "请求体不是合法 JSON"}, 400)
                    return
                if not isinstance(body, dict):
                    _json_response(self, {"ok": False, "error": "请求体必须是 JSON 对象"}, 400)
                    return
        try:
            result = fn(query, body)
        except Exception as exc:  # noqa: BLE001
            traceback.print_exc()
            _json_response(self, {"ok": False, "error": "接口异常：%s" % exc}, 500)
            return
        # 接口语义错误统一用 200 + ok:false 返回，前端只认 JSON 里的 ok
        _json_response(self, result, 200)

    def _handle_static(self, method, path):
        if method == "POST":
            _json_response(self, {"ok": False, "error": "静态资源不支持 POST"}, 405)
            return
        rel = path.lstrip("/")
        if rel == "" or rel.endswith("/"):
            rel = (rel + "index.html").lstrip("/")
        target = (WEB_DIR / rel).resolve()
        if target != WEB_DIR and WEB_DIR not in target.parents:
            _json_response(self, {"ok": False, "error": "禁止访问"}, 403)
            return
        if target.is_dir():
            target = (target / "index.html").resolve()
            if WEB_DIR not in target.parents:
                _json_response(self, {"ok": False, "error": "禁止访问"}, 403)
                return
        if not target.is_file():
            _json_response(self, {"ok": False, "error": "找不到 %s" % path}, 404)
            return
        try:
            data = target.read_bytes()
        except OSError:
            _json_response(self, {"ok": False, "error": "读取失败"}, 500)
            return
        mime = MIME_TYPES.get(target.suffix.lower())
        if mime is None:
            mime = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        self.send_response(200)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(data)))
        _no_cache(self)
        self.end_headers()
        if method != "HEAD":
            self.wfile.write(data)


class RunnerServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    SCRIPTS_DIR.mkdir(parents=True, exist_ok=True)
    address = (HOST, PORT)
    try:
        httpd = RunnerServer(address, RunnerHandler)
    except OSError as exc:
        print("无法监听 %s:%s → %s" % (HOST, PORT, exc), file=sys.stderr)
        return 1
    print("MuJoCo 运行器服务已启动：http://%s:%s/" % (HOST, PORT))
    print("静态根：%s" % WEB_DIR)
    print("项目根：%s" % WORK4_DIR)
    print("Ctrl+C 停止")
    if os.environ.get("RUNNER_OPEN_BROWSER") == "1":
        import webbrowser
        url = "http://%s:%s/" % (HOST, PORT)
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n已停止")
    finally:
        httpd.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
