/* ==========================================================================
   MuJoCo 运行器 — 共享前端基础库
   包含：API 客户端（带离线兜底）、toast、通用文件选择弹层、格式化工具
   两个页面（index.html / builder.html）都必须先引入本文件，再引入各自 js。
   API 服务默认同源；若通过 /api 不可达（例如直接双击 html 打开），
   所有调用会回落到本地演示数据，界面照常可用。
   ========================================================================== */

(function () {
  "use strict";

  const API_BASE = location.protocol === "file:" ? "http://127.0.0.1:8770" : "";

  async function req(path, opts) {
    const res = await fetch(API_BASE + path, Object.assign({
      headers: { "Content-Type": "application/json" }
    }, opts || {}));
    if (!res.ok) throw new Error("HTTP " + res.status + " " + path);
    return res.json();
  }

  /* ---------------- 演示兜底数据（后端未启动时用） ---------------- */
  const DEMO_WAYPOINTS = [
    { x: 0.500, y: -0.050, z: 1.100, hold: 0.5, gripper: "open" },
    { x: 0.500, y: -0.050, z: 0.762, hold: 0.4, gripper: "close" },
    { x: 0.520, y: 0.280, z: 0.867, hold: 0.5, gripper: "close" },
    { x: 0.520, y: 0.280, z: 0.767, hold: 0.5, gripper: "open" },
    { x: 0.520, y: 0.280, z: 0.867, hold: 0.5, gripper: "open" }
  ];

  const DEMO_RECENT = [
    { file: "scripts/pick_and_place.py", waypoints: 6, duration: 12.4, opened_at: "昨天 18:42", status: "已运行" },
    { file: "scripts/scan_sweep.py",     waypoints: 9, duration: 28.0, opened_at: "11-05 09:12", status: "草稿" },
    { file: "scripts/calib_touch.py",    waypoints: 3, duration: 4.1,  opened_at: "11-02 14:05", status: "已运行" }
  ];

  const API = {
    online: false,
    base: API_BASE,

    async health() {
      try {
        const h = await req("/api/health");
        API.online = true;
        return h;
      } catch (e) {
        API.online = false;
        return { ok: true, mujoco: "3.17.0", mink: "1.4.0", offline: true };
      }
    },

    async models() {
      try { return await req("/api/models"); }
      catch (e) {
        return {
          model: "model/ur5e.xml",
          scene: "model/scene.xml",
          models: ["model/ur5e.xml"],
          scenes: ["model/scene.xml"],
          offline: true
        };
      }
    },

    async files(kind) {
      try { return await req("/api/files?kind=" + encodeURIComponent(kind || "scene")); }
      catch (e) {
        if (kind === "model") return { items: ["model/ur5e.xml"], offline: true };
        if (kind === "script") return { items: ["scripts/pick_and_place.py"], offline: true };
        return { items: ["model/scene.xml"], offline: true };
      }
    },

    async recent() {
      try { return await req("/api/recent"); }
      catch (e) { return { items: DEMO_RECENT, offline: true }; }
    },

    async preview(payload) {
      try { return await req("/api/preview", { method: "POST", body: JSON.stringify(payload) }); }
      catch (e) {
        return {
          ok: true, offline: true,
          duration: estimateDuration(payload.waypoints || []),
          gripper_changes: countGripperChanges(payload.waypoints || []),
          reachable: true, max_ik_mm: 0.01, joint_speed: 1.12
        };
      }
    },

    async generate(payload) {
      try { return await req("/api/generate", { method: "POST", body: JSON.stringify(payload) }); }
      catch (e) {
        return {
          ok: true, offline: true,
          path: "scripts/" + (payload.name || "pick_and_place") + ".py",
          note: "后端未启动：仅生成了预览，未真正写文件"
        };
      }
    },

    async launchLayout(payload) {
      try { return await req("/api/layout/launch", { method: "POST", body: JSON.stringify(payload) }); }
      catch (e) {
        return { ok: false, error: "后端未启动，无法打开 MuJoCo 窗口", offline: true };
      }
    },

    async layoutStatus(sessionId) {
      try { return await req("/api/layout/status?sessionId=" + encodeURIComponent(sessionId)); }
      catch (e) { return { state: "unknown", offline: true }; }
    }
  };

  /* ---------------- 共用计算 ---------------- */
  const APPROACH_TIME = 5.0;
  const SEGMENT_TIME = 6.0;
  const HOLD_TAIL = 1.0;

  function estimateDuration(wps) {
    if (!wps.length) return 0;
    const holds = wps.reduce((a, w) => a + (Number(w.hold) || 0), 0);
    const travel = APPROACH_TIME + Math.max(0, wps.length - 1) * SEGMENT_TIME;
    return Math.round((travel + holds + HOLD_TAIL) * 100) / 100;
  }

  function countGripperChanges(wps) {
    let last = null, n = 0;
    for (const w of wps) {
      const g = normalizeGripper(w.gripper);
      if (g && g !== last) { n++; last = g; }
      else if (g) last = g;
    }
    return n;
  }

  function normalizeGripper(v) {
    if (v === null || v === undefined || v === "") return null;
    if (typeof v === "number") return v;
    const s = String(v).trim().toLowerCase();
    if (s === "open" || s === "close") return s;
    const n = Number(s);
    return Number.isFinite(n) ? n : null;
  }

  function fmt(n, digits) {
    const d = digits === undefined ? 3 : digits;
    if (!Number.isFinite(Number(n))) return "—";
    return Number(n).toFixed(d).replace("-", "\u2212");
  }

  function fmtTime(s) {
    return (Number(s) || 0).toFixed(2);
  }

  /* ---------------- Toast ---------------- */
  let toastHost = null;
  function ensureToastHost() {
    if (toastHost) return toastHost;
    toastHost = document.createElement("div");
    toastHost.className = "toast-host";
    document.body.appendChild(toastHost);
    return toastHost;
  }

  function toast(message, kind, ms) {
    const host = ensureToastHost();
    const el = document.createElement("div");
    el.className = "toast" + (kind ? " toast--" + kind : "");
    el.innerHTML = '<span class="toast-dot"></span><span class="toast-text"></span>';
    el.querySelector(".toast-text").textContent = message;
    host.appendChild(el);
    if (window.gsap) {
      gsap.fromTo(el, { y: 10, opacity: 0 }, { y: 0, opacity: 1, duration: .28, ease: "power2.out" });
    } else {
      el.style.opacity = "1";
    }
    const life = ms || 3600;
    setTimeout(() => {
      if (window.gsap) gsap.to(el, { y: 6, opacity: 0, duration: .22, onComplete: () => el.remove() });
      else el.remove();
    }, life);
    return el;
  }

  /* ---------------- 文件选择弹层 ---------------- */
  function pickFile(kind, current) {
    return new Promise((resolve) => {
      const overlay = document.createElement("div");
      overlay.className = "dlg-overlay";
      overlay.innerHTML =
        '<div class="dlg panel" role="dialog" aria-modal="true">' +
          '<div class="panel-head">选择 ' + ({ model: "模型 XML", scene: "场景 XML", script: "已有脚本" }[kind] || "文件") +
            '<span class="right">' + ({ model: "MODEL", scene: "SCENE", script: "PY" }[kind] || "FILE") + '</span>' +
          '</div>' +
          '<div class="dlg-body scroll" id="dlg-list"></div>' +
          '<div class="dlg-foot">' +
            '<button class="btn btn--quiet" data-act="cancel">取消</button>' +
            '<button class="btn btn--ink" data-act="ok">使用此文件</button>' +
          '</div>' +
        '</div>';
      document.body.appendChild(overlay);

      const list = overlay.querySelector("#dlg-list");
      let chosen = current || "";

      function render(items) {
        list.innerHTML = "";
        const all = (items || []).slice();
        if (current && all.indexOf(current) < 0) all.unshift(current);
        if (!all.length) {
          list.innerHTML = '<p class="hint" style="padding:16px">没有找到文件（后端可能未启动）</p>';
          return;
        }
        all.forEach((p) => {
          const row = document.createElement("button");
          row.className = "dlg-row" + (p === chosen ? " is-active" : "");
          row.innerHTML = '<span class="mono"></span><span class="dlg-check">✓</span>';
          row.querySelector(".mono").textContent = p;
          row.addEventListener("click", () => {
            chosen = p;
            list.querySelectorAll(".dlg-row").forEach((r) => r.classList.remove("is-active"));
            row.classList.add("is-active");
          });
          list.appendChild(row);
        });
      }

      API.files(kind).then((r) => render(r.items));

      function close() {
        if (window.gsap) {
          gsap.to(overlay, { opacity: 0, duration: .16, onComplete: () => overlay.remove() });
        } else overlay.remove();
      }
      overlay.querySelector('[data-act="cancel"]').addEventListener("click", () => { close(); resolve(null); });
      overlay.querySelector('[data-act="ok"]').addEventListener("click", () => { close(); resolve(chosen); });
      overlay.addEventListener("click", (e) => { if (e.target === overlay) { close(); resolve(null); } });
      document.addEventListener("keydown", function esc(e) {
        if (e.key === "Escape") { document.removeEventListener("keydown", esc); close(); resolve(null); }
      });
    });
  }

  /* ---------------- 启动时 ping 一次，用来点亮顶栏状态 ---------------- */
  document.addEventListener("DOMContentLoaded", () => {
    API.health().then((h) => {
      const dot = document.querySelector("[data-engine-status]");
      if (dot) {
        dot.textContent = h.offline ? "离线预览" : "仿真引擎就绪";
        dot.classList.toggle("is-idle", !!h.offline);
      }
      const meta = document.querySelector("[data-engine-meta]");
      if (meta && !h.offline) {
        meta.textContent = "MUJOCO " + h.mujoco + " · MINK " + h.mink;
      }
    });
  });

  window.API = API;
  window.RunnerUtil = {
    estimateDuration, countGripperChanges, normalizeGripper, fmt, fmtTime,
    APPROACH_TIME, SEGMENT_TIME, HOLD_TAIL, DEMO_WAYPOINTS, DEMO_RECENT
  };
  window.UI = { toast, pickFile };
})();
