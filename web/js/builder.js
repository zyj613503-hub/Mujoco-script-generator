/* ==========================================================================
   MuJoCo 运行器 — 副界面 · 轨迹脚本
   依赖：vendor/gsap.min.js（可选）、js/shared.js（API.* / UI.* / RunnerUtil.*）
   俯视图（FIG.02）的数据来自 /api/scene/bodies：球 / 盘 / 桌子 / 底座全部按
   当前场景 XML 里的真实坐标绘制，不在前端写死演示位置。
   ========================================================================== */
(function () {
  "use strict";

  const RM = window.matchMedia && window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  const gsap = window.gsap || null;
  const $ = (sel, root) => (root || document).querySelector(sel);
  const $$ = (sel, root) => Array.prototype.slice.call((root || document).querySelectorAll(sel));

  const DEFAULT_WP = { x: 0.5, y: -0.05, z: 1.1, hold: 0, gripper: "open" };
  /* 俯视图：桌面中心 (0,0) 落在 (cx,cy)，1 m = s px；lx 为右侧坐标标注列 */
  const FIG = { W: 810, H: 238, cx: 238, cy: 104, s: 92, lx: 346, bx: 470, by: 64, bw: 288, bh: 118 };
  const PREVIEW_DEBOUNCE = 400;
  const BODY_LABEL = { ball: "球", plate: "盘" };
  const BODY_RADIUS = { ball: 0.022, plate: 0.118 };

  const state = {
    waypoints: [Object.assign({}, DEFAULT_WP)],
    model: "model/ur5e.xml",
    scene: "model/scene.xml",
    seq: 0,             // 数据版本号
    appliedSeq: -1,     // 已应用 preview 的版本号
    preview: null,
    active: 0,          // 当前聚焦行
    polling: null,
    /* ---- 场景数据（/api/scene/bodies） ---- */
    bodies: null,       // null = 未取到，[] = 场景里没有物品
    table: null,
    base: null,
    reach: 0.85,
    figState: "idle",   // idle | loading | ok | error
    figErr: "",
    figSeq: 0,          // 丢弃过期请求
    sceneName: "",
    scriptFile: ""
  };

  let els = {};
  let previewTimer = null;
  let sceneTimer = null;

  /* ---------------- 工具 ---------------- */
  const { fmt, fmtTime, estimateDuration, countGripperChanges, normalizeGripper } = RunnerUtil;

  function parseNum(s) {
    const v = parseFloat(String(s).replace(/[\u2212\u2013\u2014]/g, "-"));
    return Number.isFinite(v) ? v : NaN;
  }
  function clamp(v, a, b) { return Math.min(b, Math.max(a, v)); }
  function esc(s) {
    return String(s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  }
  function mm(n) { return String(Math.round(Number(n) || 0)).replace(/\B(?=(\d{3})+(?!\d))/g, " "); }
  function baseName(p) { return String(p || "").split("/").pop(); }

  /* ---------------- 接口（同源；file:// 时回落到 127.0.0.1:8770） ---------------- */
  function apiBase() {
    return (window.API && API.base) || (location.protocol === "file:" ? "http://127.0.0.1:8770" : "");
  }
  async function fetchJSON(path) {
    const res = await fetch(apiBase() + path, { headers: { "Content-Type": "application/json" } });
    if (!res.ok) throw new Error("HTTP " + res.status + " " + path);
    return res.json();
  }

  /* ---------------- 数据 ---------- */
  function cloneWps() {
    return state.waypoints.map((w) => ({
      x: Number(w.x), y: Number(w.y), z: Number(w.z),
      hold: Number(w.hold) || 0,
      gripper: w.gripper || null
    }));
  }

  function metrics() {
    const wps = state.waypoints;
    const m = {
      n: wps.length,
      duration: estimateDuration(wps),
      gripperChanges: countGripperChanges(wps),
      reachable: true,
      maxIk: 0.01,
      jointSpeed: 1.12
    };
    if (state.appliedSeq === state.seq && state.preview) {
      const p = state.preview;
      if (Number.isFinite(Number(p.duration))) m.duration = Number(p.duration);
      if (Number.isFinite(Number(p.gripper_changes))) m.gripperChanges = Number(p.gripper_changes);
      m.reachable = p.reachable !== false;
      if (Number.isFinite(Number(p.max_ik_mm))) m.maxIk = Number(p.max_ik_mm);
      if (Number.isFinite(Number(p.joint_speed))) m.jointSpeed = Number(p.joint_speed);
    }
    return m;
  }

  /* ---------------- 轨迹点表 ---------- */
  function rowHTML(w, i) {
    const g = w.gripper === "close" ? "close" : (w.gripper === "open" ? "open" : "none");
    const gText = g === "none" ? "\u2014" : g;
    return '<tr data-i="' + i + '">' +
      '<td class="c-idx"><span class="idx">' + (i + 1) + "</span></td>" +
      '<td class="c-num"><input class="wp-input" data-k="x" type="text" inputmode="decimal" spellcheck="false" autocomplete="off" value="' + fmt(w.x, 3) + '" aria-label="路点 ' + (i + 1) + ' X"></td>' +
      '<td class="c-num"><input class="wp-input" data-k="y" type="text" inputmode="decimal" spellcheck="false" autocomplete="off" value="' + fmt(w.y, 3) + '" aria-label="路点 ' + (i + 1) + ' Y"></td>' +
      '<td class="c-num"><input class="wp-input" data-k="z" type="text" inputmode="decimal" spellcheck="false" autocomplete="off" value="' + fmt(w.z, 3) + '" aria-label="路点 ' + (i + 1) + ' Z"></td>' +
      '<td class="c-num"><input class="wp-input" data-k="hold" type="text" inputmode="decimal" spellcheck="false" autocomplete="off" value="' + fmt(w.hold, 1) + '" aria-label="路点 ' + (i + 1) + ' 停留秒数"></td>' +
      '<td class="c-grip"><button class="grip grip--' + g + '" type="button" data-act="grip" aria-label="切换路点 ' + (i + 1) + ' 夹爪状态">' +
        '<span class="grip-dot"></span><span class="grip-text">' + gText + "</span></button></td>" +
      '<td class="c-del"><button class="del" type="button" data-act="del" aria-label="删除路点 ' + (i + 1) + '">×</button></td>' +
      "</tr>";
  }

  function renderRows(activeIndex) {
    els.rows.innerHTML = state.waypoints.map(rowHTML).join("");
    setActiveRow(activeIndex === undefined ? 0 : activeIndex);
    if (els.body) els.body.classList.toggle("is-full", state.waypoints.length * 38 >= 190);
  }

  function setActiveRow(i) {
    state.active = i;
    $$("tr", els.rows).forEach((tr) => tr.classList.toggle("is-active", Number(tr.dataset.i) === i));
  }

  /* ---------------- 图纸 SVG ---------------- */
  function ln(x1, y1, x2, y2, color, w, dash) {
    return '<line x1="' + x1 + '" y1="' + y1 + '" x2="' + x2 + '" y2="' + y2 +
      '" style="stroke:' + color + '" stroke-width="' + (w || 1) + '"' +
      (dash ? ' stroke-dasharray="' + dash + '"' : "") + "/>";
  }
  function tx(x, y, s, size, color, ls, cls, anchor, weight) {
    return '<text x="' + x + '" y="' + y + '" font-size="' + size + '" style="fill:' + color + '"' +
      (ls ? ' letter-spacing="' + ls + '"' : "") +
      (weight ? ' font-weight="' + weight + '"' : "") +
      (cls ? ' class="' + cls + '"' : "") +
      (anchor ? ' text-anchor="' + anchor + '"' : "") + ">" + s + "</text>";
  }
  function poly(points, color, w) {
    return '<polyline points="' + points.map((p) => p[0].toFixed(1) + "," + p[1].toFixed(1)).join(" ") +
      '" fill="none" style="stroke:' + color + '" stroke-width="' + (w || 1) + '"/>';
  }

  /* 场景物品 → 绘图数据（坐标全部来自接口） */
  function bodyItems() {
    return (state.bodies || []).map((b) => {
      const pos = Array.isArray(b.pos) ? b.pos : [0, 0, 0];
      const x = Number(pos[0]) || 0;
      const y = Number(pos[1]) || 0;
      return {
        name: b.name,
        kind: b.kind || (b.name === "ball" ? "sphere" : ""),
        x: x, y: y,
        radius: Number(b.radius) > 0 ? Number(b.radius) : (BODY_RADIUS[b.name] || 0.03),
        label: (BODY_LABEL[b.name] || b.name || "?") + " " + fmt(x, 2) + ", " + fmt(y, 2)
      };
    });
  }

  /* 坐标标注：优先放右侧列；同列上下错开，避免文字叠字 */
  function layoutCallouts(items) {
    const placed = [];
    items.forEach((it) => {
      const right = it.px <= FIG.lx - 26;
      const colX = right ? FIG.lx : 124;
      let ly = clamp(it.py, 22, FIG.H - 26);
      const col = placed.filter((p) => p.right === right);
      for (let guard = 0; guard < 8; guard++) {
        const hit = col.find((p) => Math.abs(p.ly - ly) < 13);
        if (!hit) break;
        ly = hit.ly + 13;
      }
      if (ly > FIG.H - 26) {
        ly = FIG.H - 26;
        col.slice().sort((a, b) => b.ly - a.ly).forEach((p) => {
          if (p.ly > ly - 13) { p.ly = ly - 13; ly = p.ly; }
        });
      }
      placed.push({ px: it.px, py: it.py, r: it.r, text: it.text, right: right, colX: colX, ly: ly });
    });
    return placed;
  }

  function buildFig(wps) {
    const { W, H, cx, cy, s, lx, bx, by, bw, bh } = FIG;
    const A = [];

    /* 网格 */
    for (let x = 0; x <= W; x += 26) A.push(ln(x, 0, x, H, "var(--fig-grid)"));
    for (let y = 0; y <= H; y += 26) A.push(ln(0, y, W, y, "var(--fig-grid)"));

    /* 由 API 提供的桌面几何 */
    const table = state.table;
    const halfW = (table && table.size ? Math.abs(Number(table.size[0])) || 2 : 2) / 2;
    const halfH = (table && table.size ? Math.abs(Number(table.size[1])) || 2 : 2) / 2;
    const tcx = table && table.center ? Number(table.center[0]) || 0 : 0;
    const tcy = table && table.center ? Number(table.center[1]) || 0 : 0;
    const topZ = table && Number.isFinite(Number(table.top_z)) ? Number(table.top_z) : 0.75;

    const X = (v) => cx + Number(v) * s;
    const Yp = (v) => cy - Number(v) * s;

    const x0 = X(tcx - halfW), x1 = X(tcx + halfW);
    const y0 = Yp(tcy + halfH), y1 = Yp(tcy - halfH);

    A.push('<rect x="' + x0.toFixed(1) + '" y="' + y0.toFixed(1) + '" width="' + (x1 - x0).toFixed(1) +
      '" height="' + (y1 - y0).toFixed(1) + '" fill="var(--paper-card)" style="stroke:var(--line-2)"/>');

    /* 底座 + 工作半径（均以 base 的真实位置为圆心） */
    const base = state.base || { x: 0, y: 0, z: 0.75 };
    const bxp = X(base.x || 0), byp = Yp(base.y || 0);

    A.push('<circle cx="' + bxp.toFixed(1) + '" cy="' + byp.toFixed(1) + '" r="' + (state.reach * s).toFixed(1) +
      '" fill="none" style="stroke:var(--fig-ring)" stroke-dasharray="4.5 4.5"/>');
    A.push('<rect x="' + (bxp - 13).toFixed(1) + '" y="' + (byp - 7).toFixed(1) +
      '" width="26" height="14" fill="var(--paper-card)" style="stroke:var(--ink)"/>');
    A.push('<circle cx="' + bxp.toFixed(1) + '" cy="' + byp.toFixed(1) + '" r="2.5" fill="var(--ink)"/>');
    A.push(tx((bxp - 17).toFixed(1), (byp - 11).toFixed(1),
      "BASE " + fmt(base.x || 0, 2) + ", " + fmt(base.y || 0, 2), 7.5, "var(--ink-3)", 0.4, "", "end"));

    /* 坐标轴图标 */
    A.push(ln(30, 214, 58, 214, "var(--ink-3)"), ln(30, 214, 30, 186, "var(--ink-3)"));
    A.push(tx(63, 218, "X", 8, "var(--ink-3)"), tx(25, 180, "Y", 8, "var(--ink-3)"));

    /* 路点映射 */
    const P = wps.map((w) => [X(w.x), Yp(w.y)]);

    /* 图层顺序：台面 → 圈/BASE → 物品（含引线坐标）→ 顺序虚线路径 → 路点标记
       物品先画、路点后画，避免路点压住球/盘的位置；路点自身再垫一层纸色底保证编号可读 */

    /* 球 / 盘：真实坐标 + 真实半径（球 0.022 m 画出来不到 3 px，给最小可见半径） */
    const items = bodyItems();
    const marks = items.map((it) => {
      const px = X(it.x), py = Yp(it.y);
      const r = Math.max(it.radius * s, 4);
      return { it: it, px: px, py: py, r: r };
    });
    marks.forEach((m) => {
      const isBall = m.it.kind === "sphere" || m.it.name === "ball";
      A.push('<circle cx="' + m.px.toFixed(1) + '" cy="' + m.py.toFixed(1) + '" r="' + m.r.toFixed(2) +
        '" fill="' + (isBall ? "var(--fig-ball)" : "var(--paper-card)") + '" style="stroke:' +
        (isBall ? "var(--fig-ball-2)" : "var(--ink)") + '"/>');
    });

    /* 每件物品一条 1px 引线 + mono 坐标 */
    const calls = layoutCallouts(marks.map((m) => ({
      px: m.px, py: m.py, r: m.r, text: m.it.label
    })));
    calls.forEach((p) => {
      const startX = p.px + (p.right ? p.r : -p.r);
      const rail = p.right ? p.colX - 12 : p.colX + 12;
      A.push(poly([[startX, p.py], [rail, p.py], [rail, p.ly], [p.colX, p.ly]], "var(--line-2)", 1));
      A.push(tx(p.colX, (p.ly + 2.6).toFixed(1), esc(p.text), 7.5, "var(--ink-2)", 0.2, "", p.right ? "start" : "end"));
    });

    /* 顺序虚线路径 */
    for (let i = 1; i < P.length; i++) {
      A.push(ln(P[i - 1][0].toFixed(1), P[i - 1][1].toFixed(1), P[i][0].toFixed(1), P[i][1].toFixed(1), "var(--ink-3)", 1, "5 4"));
    }

    /* 路点标记：同位置归并成一个，编号 + 叠点数 ×N */
    const groups = [];
    P.forEach((p, i) => {
      const g = groups.find((q) => Math.abs(q.p[0] - p[0]) < 5 && Math.abs(q.p[1] - p[1]) < 5);
      if (g) g.count += 1;
      else groups.push({ p: p, i: i, count: 1 });
    });
    groups.forEach((g, gi) => {
      const p = g.p;
      const solid = gi === 0;
      /* 纸色方底：压在深色球/盘上也能读出编号 */
      A.push('<rect x="' + (p[0] - 8.5).toFixed(1) + '" y="' + (p[1] - 8.5).toFixed(1) +
        '" width="17" height="17" fill="var(--paper-card)"/>');
      A.push('<rect x="' + (p[0] - 6.5).toFixed(1) + '" y="' + (p[1] - 6.5).toFixed(1) + '" width="13" height="13" fill="' +
        (solid ? "var(--ink)" : "var(--paper-card)") + '" style="stroke:var(--ink)"/>');
      A.push(tx(p[0].toFixed(1), (p[1] + 3.1).toFixed(1), String(g.i + 1), 8.5,
        solid ? "var(--paper-card)" : "var(--ink)", 0, "", "middle", 500));
      if (g.count > 1) {
        /* 朱红点落在方框右下角内，避免盖住编号 */
        A.push('<circle cx="' + (p[0] - 4).toFixed(1) + '" cy="' + (p[1] + 4).toFixed(1) + '" r="2" fill="var(--accent)"/>');
        A.push('<rect x="' + (p[0] + 8).toFixed(1) + '" y="' + (p[1] - 12.5).toFixed(1) +
          '" width="13" height="9" fill="var(--paper-card)"/>');
        A.push(tx((p[0] + 9).toFixed(1), (p[1] - 5.5).toFixed(1), "\u00d7" + g.count, 7.5, "var(--ink-3)", 0, "", "start"));
      }
    });

    /* 桌面尺寸标注 + 台面高度（真实值） */
    const dimY = y1 + 9;
    A.push(ln(x0.toFixed(1), dimY.toFixed(1), x1.toFixed(1), dimY.toFixed(1), "var(--ink-3)"));
    A.push(ln(x0.toFixed(1), (dimY - 5).toFixed(1), x0.toFixed(1), (dimY + 5).toFixed(1), "var(--ink-3)"));
    A.push(ln(x1.toFixed(1), (dimY - 5).toFixed(1), x1.toFixed(1), (dimY + 5).toFixed(1), "var(--ink-3)"));
    A.push(tx(x0.toFixed(1), (dimY + 13).toFixed(1),
      mm(halfW * 2 * 1000) + " \u00b7 \u53f0\u9762 Z " + fmt(topZ, 2), 8.5, "var(--ink-3)", 0.4));

    /* ---------- 高度剖面 ---------- */
    const zs = [];
    wps.forEach((w) => { const v = Number(w.z); if (Number.isFinite(v)) zs.push(v); });
    (state.bodies || []).forEach((b) => {
      const v = Array.isArray(b.pos) ? Number(b.pos[2]) : NaN;
      if (Number.isFinite(v)) zs.push(v);
    });
    if (table) zs.push(topZ);

    let zmin = 0.70, zmax = 1.20;
    if (zs.length) {
      const lo = Math.min.apply(null, zs);
      const hi = Math.max.apply(null, zs);
      if (lo < zmin) zmin = Math.floor((lo - 0.02) * 20) / 20;
      if (hi > zmax) zmax = Math.ceil((hi + 0.02) * 20) / 20;
    }
    if (zmax - zmin < 0.1) zmax = zmin + 0.1;

    const n = wps.length;
    const Xp = (i) => (n > 1 ? bx + 28 + i * ((bw - 40) / (n - 1)) : bx + bw / 2);
    const Y = (v) => by + bh - ((clamp(Number(v) || 0, zmin, zmax) - zmin) / (zmax - zmin)) * bh;

    A.push(tx(bx, by - 14, "\u9ad8\u5ea6\u5256\u9762 \u00b7 Z", 8.5, "var(--ink-3)", 0.6, "sans"));
    A.push(ln(bx + 20, by, bx + 20, by + bh, "var(--line)"));
    A.push(ln(bx + 20, by + bh, bx + bw, by + bh, "var(--line)"));
    A.push(tx(bx - 6, by + 6, fmt(zmax, 2), 7.5, "var(--ink-3)"));
    A.push(tx(bx - 6, by + bh - 2, fmt(zmin, 2), 7.5, "var(--ink-3)"));

    /* 台面高度参考线 */
    if (topZ >= zmin && topZ <= zmax) {
      const ty = Y(topZ);
      A.push(ln(bx + 20, ty.toFixed(1), bx + bw, ty.toFixed(1), "var(--fig-ring)", 1, "3 3"));
      A.push(tx(bx + bw - 2, (ty - 3).toFixed(1), "\u53f0\u9762 " + fmt(topZ, 2), 7, "var(--ink-3)", 0, "", "end"));
    }

    const pts = wps.map((w, i) => [Xp(i), Y(w.z)]);

    /* 单点时用虚线把该点引到基线，避免空图 */
    if (n === 1) A.push(ln(Xp(0), by + bh, Xp(0), pts[0][1], "var(--fig-ring)", 1, "3 3"));
    if (n > 1) {
      A.push('<polyline points="' + pts.map((p) => p[0].toFixed(1) + "," + p[1].toFixed(1)).join(" ") +
        '" fill="none" style="stroke:var(--line-2)" stroke-width="1.5"/>');
    }
    pts.forEach((p) => {
      A.push('<circle cx="' + p[0].toFixed(1) + '" cy="' + p[1].toFixed(1) + '" r="4" fill="var(--paper-card)" style="stroke:var(--ink)"/>');
    });
    /* 横轴序号：≤8 个全标，更多只标首 / 当前 / 尾 */
    const ticks = n <= 8 ? pts.map((_, i) => i) : [0, state.active || 0, n - 1];
    ticks.filter((v, i, arr) => arr.indexOf(v) === i && v >= 0 && v < n).forEach((i) => {
      A.push(tx((Xp(i) - 3).toFixed(1), by + bh + 12, String(i + 1), 7.5, "var(--ink-3)"));
    });
    if (n <= 5) A.push(tx(n > 1 ? bx + 58 : bx + 28, by + bh + 12, "\u8def\u70b9", 7.5, "var(--ink-3)", 0, "sans"));

    const bodyCount = items.length;
    return '<svg width="810" height="238" viewBox="0 0 810 238" role="img" aria-label="\u684c\u9762\u4fef\u89c6\uff1a' +
      bodyCount + ' \u4ef6\u7269\u54c1\u4e0e ' + n + ' \u4e2a\u8def\u70b9\uff1b\u53f3\u4fa7\u4e3a\u9ad8\u5ea6\u5256\u9762">' +
      A.join("") + "</svg>";
  }

  function renderFig() {
    if (!els.fig) return;
    els.fig.innerHTML = buildFig(state.waypoints);
    if (els.figScene) {
      const label = state.figState === "error"
        ? "\u5750\u6807\u4e0d\u53ef\u7528"
        : (state.figState === "loading" && !state.bodies
            ? "\u8bfb\u53d6\u573a\u666f\u5750\u6807\u2026"
            : (state.sceneName ? "SCENE \u00b7 " + baseName(state.sceneName) : "\u2014"));
      els.figScene.textContent = label;
      els.figScene.title = state.figState === "error"
        ? (state.figErr || "\u65e0\u6cd5\u8bfb\u53d6\u573a\u666f\u5750\u6807")
        : (state.scene || "");
      els.figScene.classList.toggle("is-bad", state.figState === "error");
    }
  }

  /* ---------------- 场景数据加载 ---------------- */
  function setScene(path) {
    if (!path) return;
    state.scene = path;
    state.sceneName = path;
    if (els.scene) els.scene.value = path;
  }

  function applySceneData(data) {
    const table = data.table;
    state.table = table && Array.isArray(table.size)
      ? { center: Array.isArray(table.center) ? table.center : [0, 0], size: table.size, top_z: table.top_z }
      : null;
    state.base = data.base || null;
    state.reach = Number(data.reach) > 0 ? Number(data.reach) : 0.85;
    state.bodies = (Array.isArray(data.bodies) ? data.bodies : []).map((b) => ({
      name: String(b.name || ""),
      kind: String(b.kind || ""),
      pos: (Array.isArray(b.pos) ? b.pos : [0, 0, 0]).map(Number),
      radius: Number(b.radius) > 0 ? Number(b.radius) : undefined,
      approximate: !!b.approximate
    }));
  }

  async function loadBodies(scene) {
    const target = String(scene || "").trim();
    if (!target) return;
    const seq = ++state.figSeq;
    state.figState = "loading";
    state.sceneName = target;
    renderFig();
    let res = null;
    try { res = await fetchJSON("/api/scene/bodies?scene=" + encodeURIComponent(target)); } catch (e) { res = null; }
    if (seq !== state.figSeq) return;            // 输入框又变了，丢弃这次结果
    if (res && res.ok) {
      applySceneData(res);
      state.figState = "ok";
      state.figErr = "";
    } else {
      state.bodies = null;
      state.table = null;
      state.base = null;
      state.figState = "error";
      state.figErr = (res && (res.error || res.detail)) || "场景坐标接口不可用";
    }
    renderFig();
  }

  async function getLayoutCurrent() {
    try {
      const r = await fetchJSON("/api/layout/current");
      return r && r.ok !== false ? r : null;
    } catch (e) { return null; }
  }

  /* ---------------- 摘要/状态同步 ---------------- */
  function updateChrome() {
    const m = metrics();
    const dur = fmtTime(m.duration);

    els.headCount.textContent = m.n + " 路点 · " + dur + " S · " + m.gripperChanges + " 次夹爪动作";
    els.wptSub.textContent = m.n + " 行 · 自上而下依次执行";
    els.outN.textContent = String(m.n);
    els.outDur.textContent = dur + " s";
    els.outGrip.textContent = m.gripperChanges + " 次";

    if (m.reachable) {
      els.check.classList.remove("is-bad");
      els.checkText.textContent = "校验 · 全部路点可达";
    } else {
      els.check.classList.add("is-bad");
      els.checkText.textContent = "校验 · 有路点超出工作半径";
    }
    els.outNote.textContent = "IK 最大误差 " + fmt(m.maxIk, 2) + " MM · 关节速度 " + fmt(m.jointSpeed, 2) + " RAD/S";
  }

  /* ---------------- preview 防抖 ---------------- */
  function schedulePreview() {
    clearTimeout(previewTimer);
    previewTimer = setTimeout(runPreview, PREVIEW_DEBOUNCE);
  }

  async function runPreview() {
    clearTimeout(previewTimer);
    const seq = state.seq;
    let res = null;
    try {
      res = await API.preview({ waypoints: cloneWps(), model: state.model, scene: state.scene });
    } catch (e) {
      res = null;
    }
    if (seq !== state.seq) return;
    if (res && res.ok !== false) {
      state.preview = res;
      state.appliedSeq = seq;
    } else {
      state.preview = null;
      state.appliedSeq = -1;
    }
    updateChrome();
  }

  function dataChanged() {
    state.seq += 1;
    updateChrome();
    renderFig();
    schedulePreview();
  }

  /* ---------------- 行操作 ---------------- */
  function bindRows() {
    els.rows.addEventListener("input", (e) => {
      const input = e.target.closest(".wp-input");
      if (!input) return;
      const tr = input.closest("tr");
      const i = Number(tr.dataset.i);
      const k = input.dataset.k;
      const v = parseNum(input.value);
      if (!Number.isFinite(v)) return;
      state.waypoints[i][k] = v;
      dataChanged();
    });

    els.rows.addEventListener("blur", (e) => {
      const input = e.target.closest(".wp-input");
      if (!input) return;
      const tr = input.closest("tr");
      const i = Number(tr.dataset.i);
      const w = state.waypoints[i];
      if (!w) return;
      input.value = input.dataset.k === "hold" ? fmt(w.hold, 1) : fmt(w[input.dataset.k], 3);
    }, true);

    els.rows.addEventListener("focusin", (e) => {
      const tr = e.target.closest("tr");
      if (!tr) return;
      const i = Number(tr.dataset.i);
      if (i === state.active) return;
      setActiveRow(i);
      if (state.waypoints.length > 8) renderFig();
    });

    els.rows.addEventListener("click", (e) => {
      const btn = e.target.closest("button[data-act]");
      if (!btn) return;
      const tr = btn.closest("tr");
      const i = Number(tr.dataset.i);
      if (btn.dataset.act === "grip") toggleGrip(i, btn);
      else if (btn.dataset.act === "del") deleteRow(i);
    });
  }

  function toggleGrip(i, btn) {
    const w = state.waypoints[i];
    if (!w) return;
    w.gripper = w.gripper === "close" ? "open" : (w.gripper === "open" ? "close" : "open");
    const g = w.gripper;
    btn.classList.toggle("grip--close", g === "close");
    btn.classList.toggle("grip--none", g === "none");
    btn.querySelector(".grip-text").textContent = g;
    dataChanged();
  }

  function deleteRow(i) {
    if (state.waypoints.length <= 1) {
      UI.toast("至少保留 1 个路点，已重置为默认坐标", "warn");
      state.waypoints[0] = Object.assign({}, DEFAULT_WP);
      state.seq += 1;
      renderRows(0);
      dataChanged();
      return;
    }
    state.waypoints.splice(i, 1);
    state.seq += 1;
    renderRows(Math.min(i, state.waypoints.length - 1));
    dataChanged();
  }

  function addRow() {
    const last = state.waypoints[state.waypoints.length - 1] || DEFAULT_WP;
    state.waypoints.push({
      x: last.x, y: last.y, z: last.z,
      hold: 0,
      gripper: last.gripper === "close" ? "close" : (last.gripper === "open" ? "open" : null)
    });
    state.seq += 1;
    renderRows(state.waypoints.length - 1);
    dataChanged();

    els.body.scrollTop = els.body.scrollHeight;
    const row = els.rows.lastElementChild;
    if (row && gsap && !RM) gsap.from(row, { y: 8, opacity: 0, duration: 0.25, ease: "power2.out" });
    if (row) {
      const input = row.querySelector('.wp-input[data-k="x"]');
      if (input) { input.focus({ preventScroll: true }); input.select(); }
    }
  }

  /* ---------------- 布置场景 ---------------- */
  function resetLayoutBtn() {
    const btn = els.btnLayout;
    btn.disabled = false;
    btn.querySelector(".btn-label").textContent = "布置场景";
  }

  function adoptBodies(list) {
    /* 保存接口顺带回的 bodies：先立即重绘，随后再用 /api/scene/bodies 补齐 table/base */
    if (!Array.isArray(list) || !list.length || !state.bodies) return;
    const byName = {};
    state.bodies.forEach((b) => { byName[b.name] = b; });
    list.forEach((b) => {
      const cur = byName[b.name];
      if (cur && Array.isArray(b.pos)) cur.pos = b.pos.map(Number);
    });
  }

  async function launchLayout() {
    const btn = els.btnLayout;
    if (btn.disabled) return;
    if (state.polling) { clearInterval(state.polling); state.polling = null; }

    btn.disabled = true;
    btn.querySelector(".btn-label").textContent = "场景布置中…";

    let res = null;
    try {
      res = await API.launchLayout({ model: state.model, scene: state.scene });
    } catch (e) {
      res = null;
    }
    if (!res || !res.ok || !res.sessionId) {
      UI.toast((res && res.error) || "无法打开 MuJoCo 窗口", "err", 4200);
      resetLayoutBtn();
      return;
    }

    UI.toast("MuJoCo 窗口已打开：拖动球 / 盘子后按 S 保存", "ok", 4200);

    const sessionId = res.sessionId;
    const started = Date.now();
    let tries = 0;

    state.polling = setInterval(async () => {
      tries += 1;
      if (Date.now() - started > 10 * 60 * 1000) {
        clearInterval(state.polling);
        state.polling = null;
        resetLayoutBtn();
        UI.toast("等待场景保存超时（10 分钟）", "warn");
        return;
      }
      let st = null;
      try { st = await API.layoutStatus(sessionId); } catch (e) { st = null; }
      const s = st && st.state;

      if (s === "saved") {
        clearInterval(state.polling);
        state.polling = null;
        resetLayoutBtn();
        const savedScene = st.savedScene;
        if (savedScene) setScene(savedScene);
        adoptBodies(st.bodies);
        state.figState = "loading";
        renderFig();
        await loadBodies(state.scene);
        UI.toast("布置已保存：" + (savedScene || "scene.xml"), "ok", 4200);
        schedulePreview();
      } else if (s === "closed") {
        clearInterval(state.polling);
        state.polling = null;
        resetLayoutBtn();
        UI.toast("布置窗口已关闭", "warn");
      } else if (s === "error") {
        clearInterval(state.polling);
        state.polling = null;
        resetLayoutBtn();
        const txt = String((st && (st.log || st.error)) || "");
        if (/窗口被关闭|exitCode\s*-15|-15/.test(txt)) {
          UI.toast("布置窗口被强制关闭，未保存", "err", 4600);
        } else {
          UI.toast("场景布置出错" + (st && st.error ? "：" + st.error : ""), "err", 4200);
        }
      } else if (tries >= 3 && (!st || st.offline || s === "unknown")) {
        clearInterval(state.polling);
        state.polling = null;
        resetLayoutBtn();
        UI.toast("无法获取场景窗口状态（后端未就绪）", "warn", 4200);
      }
    }, 1500);
  }

  /* ---------------- 加载已有脚本 ---------------- */
  function readLastScript() {
    try {
      const raw = localStorage.getItem("lastScript") ||
        localStorage.getItem("lastScriptFile") ||
        localStorage.getItem("mj_last_script") || "";
      if (!raw) return "";
      if (raw.charAt(0) === "{") {
        try { const o = JSON.parse(raw); return o.file || o.path || ""; } catch (e) { return ""; }
      }
      return String(raw);
    } catch (e) { return ""; }
  }

  function normalizeScriptPath(p) {
    let s = String(p || "").trim().replace(/^["']|["']$/g, "");
    if (!s) return "";
    if (/^scripts\//.test(s)) return s;
    const name = s.split("/").pop();
    return name ? "scripts/" + name : "";
  }

  function normWp(w) {
    const g = normalizeGripper(w.gripper);
    let grip = null;
    if (g === "open" || g === "close") grip = g;
    else if (typeof g === "number") grip = g >= 0.5 ? "close" : "open";
    return {
      x: Number(w.x) || 0,
      y: Number(w.y) || 0,
      z: Number(w.z) || 0,
      hold: Number(w.hold) || 0,
      gripper: grip
    };
  }

  async function loadScript(file) {
    const path = normalizeScriptPath(file);
    if (!path) return false;
    let res = null;
    try { res = await fetchJSON("/api/script/load?file=" + encodeURIComponent(path)); } catch (e) { res = null; }
    if (!res || res.ok === false) {
      UI.toast("无法加载脚本：" + ((res && res.error) || "后端未响应"), "err", 4200);
      return false;
    }
    const list = Array.isArray(res.waypoints) ? res.waypoints.slice(0, 20) : [];
    if (list.length) {
      state.waypoints = list.map(normWp);
      state.seq += 1;
      renderRows(0);
    }
    if (res.scene) setScene(res.scene);
    if (res.model) { state.model = res.model; if (els.model) els.model.value = res.model; }
    const name = res.name || baseName(path).replace(/\.py$/i, "");
    if (els.name && name) els.name.value = name;
    state.scriptFile = res.file || path;
    state.sceneName = state.scene;
    updateChrome();
    renderFig();
    schedulePreview();
    UI.toast("已载入 " + (res.file || path), "ok", 4200);
    return true;
  }

  /* ---------------- 生成脚本 ---------------- */
  function showPath(path) {
    els.cmdPath.textContent = path;
    els.cmdPath.classList.add("is-on");
    els.cmdPath.title = path;
  }

  async function generate() {
    const btn = els.btnGenerate;
    if (btn.disabled) return;
    const name = els.name.value.trim().replace(/\.py$/i, "");
    if (!name) {
      UI.toast("请填写脚本文件名", "warn");
      els.name.focus();
      return;
    }
    const old = btn.textContent;
    btn.disabled = true;
    btn.textContent = "生成中…";
    let res = null;
    try {
      res = await API.generate({
        waypoints: cloneWps(),
        model: state.model,
        scene: state.scene,
        name: name
      });
    } catch (e) {
      res = null;
    }
    btn.disabled = false;
    btn.textContent = old;

    if (res && res.ok) {
      const path = res.path || ("scripts/" + name + ".py");
      UI.toast("脚本已生成：" + path, "ok", 4200);
      showPath(path);
    } else {
      UI.toast("生成失败：" + ((res && res.error) || "后端未响应"), "err", 4200);
    }
  }

  /* ---------------- 进入动效 ---------------- */
  function animateIn() {
    /* ?static=1 供截图/自动化直接跳到终态（不改变正常交互动效） */
    if (!gsap || RM || new URLSearchParams(location.search).has("static")) {
      document.body.classList.add("no-motion");
      return;
    }
    const tl = gsap.timeline({ defaults: { ease: "power2.out", duration: 0.5 } });
    tl.from(".head-eyebrow", { y: 8, opacity: 0, duration: 0.45 }, 0)
      .from(".page-title", { y: 10, opacity: 0, duration: 0.5 }, 0.05)
      .from(".head-meta", { y: 8, opacity: 0, duration: 0.45 }, 0.1)
      .from(".fig-panel", { y: 12, opacity: 0, duration: 0.55 }, 0.14)
      .from(".side-panel", { y: 12, opacity: 0, stagger: 0.05 }, 0.18)
      .from(".wpt-head", { y: 8, opacity: 0, duration: 0.4 }, 0.24)
      .from(".wpt-table thead th", { opacity: 0, duration: 0.3, stagger: 0.02 }, 0.3)
      .from(".wpt-table tbody tr", { y: 8, opacity: 0, duration: 0.35, stagger: 0.04 }, 0.32)
      .from(".wpt-add", { opacity: 0, duration: 0.3 }, 0.45)
      .from(".cmdbar-inner > *", { y: 6, opacity: 0, duration: 0.4, stagger: 0.04 }, 0.2);
  }

  /* ---------------- 初始化 ---------------- */
  function bindChrome() {
    els.addBtn = $("#wpt-add");
    els.addBtn.addEventListener("click", addRow);

    $$("[data-pick]").forEach((btn) => {
      btn.addEventListener("click", async () => {
        const kind = btn.dataset.pick;
        const input = kind === "model" ? els.model : els.scene;
        const picked = await UI.pickFile(kind, input.value.trim());
        if (!picked) return;
        input.value = picked;
        if (kind === "model") {
          state.model = picked;
          schedulePreview();
        } else {
          setScene(picked);
          await loadBodies(picked);          // 选新场景后立即重取坐标并重绘
          schedulePreview();
        }
      });
    });

    els.model.addEventListener("input", () => { state.model = els.model.value.trim(); schedulePreview(); });

    /* 场景 XML 改动 → 重新取 bodies 并重绘俯视图 */
    els.scene.addEventListener("input", () => {
      const v = els.scene.value.trim();
      state.scene = v;
      state.sceneName = v;
      clearTimeout(sceneTimer);
      sceneTimer = setTimeout(() => loadBodies(v), 350);
      schedulePreview();
    });

    els.btnLayout.addEventListener("click", launchLayout);
    els.btnGenerate.addEventListener("click", generate);
    els.name.addEventListener("keydown", (e) => { if (e.key === "Enter") generate(); });
  }

  async function boot() {
    els = {
      fig: $("#fig-art"),
      figScene: $("#fig-scene"),
      rows: $("#wpt-rows"),
      body: $("#wpt-body"),
      headCount: $("#head-count"),
      wptSub: $("#wpt-sub"),
      outN: $("#out-n"),
      outDur: $("#out-dur"),
      outGrip: $("#out-grip"),
      check: $("#out-check"),
      checkText: $("#out-check-text"),
      outNote: $("#out-note"),
      model: $("#model-xml"),
      scene: $("#scene-xml"),
      btnLayout: $("#btn-layout"),
      name: $("#script-name"),
      cmdPath: $("#cmd-path"),
      btnGenerate: $("#btn-generate")
    };

    renderRows(0);
    updateChrome();
    renderFig();
    bindRows();
    bindChrome();

    /* 1) 上一次布置：has:true → 场景指向保存出来的 scene_layout_*.xml */
    const layout = await getLayoutCurrent();
    if (layout && layout.has && layout.savedScene) setScene(layout.savedScene);
    else state.sceneName = state.scene;

    /* 2) ?script= 优先；否则 localStorage.lastScript（主界面写入） */
    const paramScript = new URLSearchParams(location.search).get("script");
    const scriptFile = paramScript || readLastScript();
    if (scriptFile) await loadScript(scriptFile);

    /* 3) 按最终场景取真实坐标，重绘 FIG.02 */
    await loadBodies(state.scene);

    /* 4) 首屏同步一次后端数据 */
    runPreview();
    animateIn();
  }

  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", boot);
  else boot();
})();
