/* ==========================================================================
   主界面 — 交互与入场动效
   依赖：vendor/gsap.min.js、js/shared.js（API / UI / RunnerUtil）
   ========================================================================== */

(function () {
  "use strict";

  var REDUCE = window.matchMedia && window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  var HAS_GSAP = !!window.gsap;

  var ARROW_SVG =
    '<svg width="16" height="9" viewBox="0 0 16 9" aria-hidden="true">' +
    '<path d="M0.5 4.5H15M11.2 1 15 4.5 11.2 8" fill="none" stroke="currentColor" stroke-width="1"></path>' +
    "</svg>";

  /* ---------------- 入口 01 / 02 ---------------- */
  function bindEntries() {
    var gen = document.getElementById("entry-generate");
    var load = document.getElementById("entry-load");

    if (gen) {
      gen.addEventListener("click", function () {
        location.href = "builder.html";
      });
    }

    if (load && window.UI && typeof UI.pickFile === "function") {
      load.addEventListener("click", function () {
        UI.pickFile("script", "").then(function (path) {
          if (!path) return;
          try { localStorage.setItem("lastScript", path); } catch (e) { /* 隐私模式忽略 */ }
          location.href = "builder.html?script=" + encodeURIComponent(path);
        });
      });
    }
  }

  /* ---------------- 最近打开 ---------------- */
  function fmtDuration(v) {
    if (v === null || v === undefined || v === "") return "\u2014";
    if (typeof v === "number") return v.toFixed(1) + " S";
    var s = String(v).trim();
    return /[a-z\u4e00-\u9fa5]/i.test(s) ? s : s + " S";
  }

  function isDraft(status) {
    return /草稿|draft|未生成/i.test(String(status || ""));
  }

  function buildRow(item) {
    var row = document.createElement("button");
    row.type = "button";
    row.className = "recent-row";
    row.setAttribute("data-file", item.file || "");

    var file = document.createElement("span");
    file.className = "c-file";
    file.textContent = item.file || "\u2014";
    file.title = item.file || "";

    var meta = document.createElement("span");
    meta.className = "c-meta";
    var wp = item.waypoints === undefined || item.waypoints === null
      ? "\u2014" : String(item.waypoints) + " 路点";
    meta.textContent = [wp, fmtDuration(item.duration), item.opened_at || "\u2014"].join(" \u00b7 ");

    var status = document.createElement("span");
    status.className = "c-status" + (isDraft(item.status) ? " is-draft" : "");
    status.textContent = item.status || "\u2014";

    var go = document.createElement("span");
    go.className = "c-go";
    go.innerHTML = ARROW_SVG;

    row.appendChild(file);
    row.appendChild(meta);
    row.appendChild(status);
    row.appendChild(go);

    row.addEventListener("click", function () {
      if (!item.file) return;
      location.href = "builder.html?script=" + encodeURIComponent(item.file);
    });

    return row;
  }

  function renderRecent(items) {
    var host = document.getElementById("recent-rows");
    var meta = document.getElementById("recent-meta");
    if (!host) return [];

    host.innerHTML = "";
    var list = (items || []).slice();

    if (meta) {
      meta.textContent = list.length + " 个脚本 \u00b7 WORKCELL-01";
    }

    if (!list.length) {
      var empty = document.createElement("p");
      empty.className = "recent-empty";
      empty.textContent = "还没有脚本记录。生成第一个脚本后会出现在这里。";
      host.appendChild(empty);
      return [];
    }

    return list.map(function (item) {
      var row = buildRow(item);
      host.appendChild(row);
      return row;
    });
  }

  /* ---------------- 动效 ---------------- */
  function animateIntro(rows) {
    if (!HAS_GSAP || REDUCE) return;

    gsap.from([".intro .eyebrow-rule", ".intro-title", ".intro-desc", "#entry-generate", "#entry-load"], {
      y: 8,
      opacity: 0,
      duration: 0.5,
      ease: "power2.out",
      stagger: 0.06,
      clearProps: "opacity,transform"
    });

    gsap.from(".recent-head", {
      opacity: 0,
      y: 8,
      duration: 0.5,
      ease: "power2.out",
      delay: 0.26,
      clearProps: "opacity,transform"
    });

    if (rows && rows.length) {
      gsap.from(rows, {
        y: 8,
        opacity: 0,
        duration: 0.5,
        ease: "power2.out",
        stagger: 0.06,
        delay: 0.3,
        clearProps: "opacity,transform"
      });
    }
  }

  /* ---------------- 启动 ---------------- */
  function init() {
    bindEntries();

    API.recent().then(function (res) {
      animateIntro(renderRecent(res && res.items));
    }).catch(function () {
      animateIntro(renderRecent([]));
    });
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init);
  } else {
    init();
  }
})();
