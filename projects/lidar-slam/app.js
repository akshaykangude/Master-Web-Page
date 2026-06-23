// app.js — LiDAR SLAM Simulator (browser port)
// Full feature-parity port of lidar_slam_ui.py: toolbar tools, room editor,
// obstacle add/move/resize/delete, custom walls, LiDAR raycasting with
// configurable range/rays/FOV, occupancy-grid SLAM mapping, A* navigation
// with live replanning + proximity safety zones, scan recording, map
// save (downloads .json) / load (file picker), and calibration/relocalization
// via scan-pose matching. Physics/SLAM math runs in Python via Pyodide
// (sim-core.js); this file owns the Canvas rendering, UI, and game loop.

(function () {
  "use strict";

  // ═══════════════════════════════════════════════════════════════════════
  // CONSTANTS
  // ═══════════════════════════════════════════════════════════════════════
  const WORLD_HALF = 9.0;
  const canvas = document.getElementById("simCanvas");
  const ctx = canvas.getContext("2d");
  const matrixCanvas = document.getElementById("distMatrix");
  const mctx = matrixCanvas.getContext("2d");

  function css(v) { return getComputedStyle(document.documentElement).getPropertyValue(v).trim(); }
  let COL = null;
  function resolveColors() {
    COL = {
      bg: css("--sim-bg"), bg2: css("--sim-bg2"), grid: css("--sim-grid"),
      wall: css("--sim-wall"), roomWall: css("--sim-room-wall"),
      hit: css("--sim-hit"), ray: css("--sim-ray"),
      robot: css("--sim-robot"), robotB: css("--sim-robot-b"),
      free: css("--sim-free"), occ: css("--sim-occ"),
      path: css("--sim-path"), goal: css("--sim-goal"),
      match: css("--sim-match"), calib: css("--sim-calib"),
      build: css("--sim-build"), obstacle: css("--sim-obstacle"),
      danger: css("--sim-danger"), warn: css("--sim-warn"),
      zoneGreen: css("--zone-green"), zoneYellow: css("--zone-yellow"), zoneRed: css("--zone-red"),
      textDim: css("--text-faint"), textHi: css("--text"),
    };
  }

  const OBJ_COLORS = [
    "#3786dd", "#1d9e75", "#d85a30", "#d4536e",
    "#7f77dd", "#ef9f27", "#e24b4a", "#639922",
    "#00b4c8", "#b47830", "#8c50c8", "#3ca064",
  ];

  // ═══════════════════════════════════════════════════════════════════════
  // STATE
  // ═══════════════════════════════════════════════════════════════════════
  const S = {
    roomW: 7.0, roomH: 5.5,
    rx: 0.0, ry: 0.0, rtheta: 0.0,
    robotR: 0.25,

    numRays: 360, maxRange: 8.0, fov: 360, scanHz: 10,
    distances: new Array(360).fill(null),
    scanCount: 0, lastScanT: 0,

    objects: [], nextId: 0, colorIdx: 0,
    customWalls: [], // [x1,y1,x2,y2]

    savedScans: [], recording: false,

    goal: null, path: [], pathIdx: 0, following: false, goalUnreachable: false,
    // Bug2 algorithm state
    bug2Mode: false,          // true = using Bug2 reactive nav (no A* path)
    bug2State: "IDLE",        // "IDLE" | "MOVE_TO_GOAL" | "BOUNDARY_FOLLOW"
    bug2HitPt: null,          // point where we first hit the obstacle
    bug2HitDistToGoal: null,  // dist from hit point to goal (must beat this to leave boundary)
    bug2MLineAngle: null,     // direction from start -> goal (fixed)
    bug2MLineStart: null,     // start position when Bug2 was initiated
    bug2BoundarySteps: 0,     // steps taken in boundary follow
    bug2MaxBoundarySteps: 2000, // give up after this many steps
    bug2BoundaryDir: 1,       // +1 = follow right wall, -1 = follow left wall
    bug2NoPath: false,        // set true when no path detected

    calibMode: false, calibScans: [], matchResult: null,

    tool: "NAVIGATE",
    selected: null, dragObj: null, dragOff: [0, 0],
    wallStart: null, wallPreview: null,
    resizeHandle: null,

    proxGreen: 3.0, proxYellow: 1.5, proxRed: 0.6, closestFwd: null,

    moveSpeed: 0.08,
    objR: 0.5, objW: 1.2, objH: 0.8,

    mapDirty: true,
    status: "Loading…",
  };

  let pyodide = null;
  let scene; // canvas geometry, computed on resize

  // ═══════════════════════════════════════════════════════════════════════
  // SCENE OBJECTS
  // ═══════════════════════════════════════════════════════════════════════
  function nextColor() {
    const c = OBJ_COLORS[S.colorIdx % OBJ_COLORS.length];
    S.colorIdx++;
    return c;
  }
  function addCircle(x, y, r) {
    S.objects.push({ id: S.nextId, kind: "circle", x, y, r, color: nextColor(), label: "C" + S.nextId });
    S.nextId++; S.mapDirty = true;
  }
  function addBox(x, y, w, h) {
    S.objects.push({ id: S.nextId, kind: "box", x, y, w, h, color: nextColor(), label: "B" + S.nextId });
    S.nextId++; S.mapDirty = true;
  }
  function hitObject(o, wx, wy) {
    if (o.kind === "circle") return Math.hypot(wx - o.x, wy - o.y) <= o.r + 0.3;
    return wx >= o.x - o.w / 2 - 0.2 && wx <= o.x + o.w / 2 + 0.2 &&
           wy >= o.y - o.h / 2 - 0.2 && wy <= o.y + o.h / 2 + 0.2;
  }
  function defaultScene() {
    S.objects = []; S.nextId = 0; S.colorIdx = 0;
    addCircle(3, 2, 0.5); addCircle(-2, 3, 0.4);
    addBox(2, -2.5, 1.2, 0.8); addCircle(-3, -2, 0.6);
  }

  function allWalls() {
    const rw = S.roomW, rh = S.roomH;
    const room = [
      [-rw, -rh, rw, -rh], [rw, -rh, rw, rh],
      [rw, rh, -rw, rh], [-rw, rh, -rw, -rh],
    ];
    return room.concat(S.customWalls.map(w => w.slice()));
  }
  function objectsForPy() {
    return S.objects.map(o => o.kind === "circle"
      ? { kind: "circle", x: o.x, y: o.y, r: o.r }
      : { kind: "box", x: o.x, y: o.y, w: o.w, h: o.h });
  }

  function distPtSeg(px, py, x1, y1, x2, y2) {
    const dx = x2 - x1, dy = y2 - y1;
    if (dx === 0 && dy === 0) return Math.hypot(px - x1, py - y1);
    let t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy);
    t = Math.max(0, Math.min(1, t));
    return Math.hypot(px - (x1 + t * dx), py - (y1 + t * dy));
  }

  // ═══════════════════════════════════════════════════════════════════════
  // PYTHON BRIDGE
  // ═══════════════════════════════════════════════════════════════════════
  let pyFns = {};
  function bindPyFns() {
    const names = ["occ_reset", "occ_update", "occ_export", "occ_import",
      "run_scan", "run_astar", "run_score_pose", "run_localize_json", "run_cast_single"];
    for (const n of names) pyFns[n] = pyodide.globals.get(n);
  }

  function doScan() {
    const walls = JSON.stringify(allWalls());
    const objs = JSON.stringify(objectsForPy());
    const resultJson = pyFns.run_scan(S.rx, S.ry, S.rtheta, S.numRays, S.fov, walls, objs, S.maxRange);
    const result = JSON.parse(resultJson);
    S.distances = result.distances;
    S.mapDirty = true;
    S.scanCount++;

    const fovHalf = S.fov / 2.0;
    const fwd = [];
    for (let i = 0; i < 360; i++) {
      const d = S.distances[i];
      if (d === null) continue;
      const rel = (((i - S.rtheta + 540) % 360) - 180);
      if (Math.abs(rel) <= fovHalf) fwd.push(d);
    }
    S.closestFwd = fwd.length ? Math.min(...fwd) : null;

    if (S.recording) {
      S.savedScans.push({ x: S.rx, y: S.ry, theta: S.rtheta, dists: S.distances.map(d => d === null ? -1 : d) });
    }
  }

  function castSingle(ox, oy, angleDeg) {
    const walls = JSON.stringify(allWalls());
    const objs = JSON.stringify(objectsForPy());
    return JSON.parse(pyFns.run_cast_single(ox, oy, angleDeg, walls, objs, S.maxRange));
  }

  function planPath(start, goal, robotR) {
    const p = JSON.parse(pyFns.run_astar(JSON.stringify(start), JSON.stringify(goal), robotR));
    return p;
  }

  // ═══════════════════════════════════════════════════════════════════════
  // COORDINATE TRANSFORMS
  // ═══════════════════════════════════════════════════════════════════════
  function computeScene() {
    const w = canvas.width, h = canvas.height;
    const scale = w / (WORLD_HALF * 2);
    scene = { w, h, scale, scx: w / 2, scy: h / 2 };
  }
  function w2s(wx, wy) { return [scene.scx + wx * scene.scale, scene.scy - wy * scene.scale]; }
  function s2w(sx, sy) { return [(sx - scene.scx) / scene.scale, -(sy - scene.scy) / scene.scale]; }

  function roomHandles() { return [[S.roomW, 0], [0, S.roomH], [-S.roomW, 0], [0, -S.roomH]]; }
  function roomHandleHit(wx, wy) {
    const hs = roomHandles();
    for (let i = 0; i < hs.length; i++) {
      if (Math.hypot(wx - hs[i][0], wy - hs[i][1]) < 0.5) return i;
    }
    return null;
  }

  // ═══════════════════════════════════════════════════════════════════════
  // OCCUPANCY MAP CACHE (rendered to an offscreen canvas, redrawn only when dirty)
  // ═══════════════════════════════════════════════════════════════════════
  let mapCacheCanvas = document.createElement("canvas");
  let mapCacheCtx = mapCacheCanvas.getContext("2d");

  function redrawMapCache() {
    mapCacheCanvas.width = canvas.width;
    mapCacheCanvas.height = canvas.height;
    mapCacheCtx.fillStyle = COL.bg;
    mapCacheCtx.fillRect(0, 0, canvas.width, canvas.height);

    const exported = JSON.parse(pyFns.occ_export());
    const n = exported.n;
    const GRID_RES = (WORLD_HALF * 2) / n;
    const cs = Math.max(1, scene.scale * GRID_RES);

    mapCacheCtx.fillStyle = COL.free;
    for (const idx of exported.free) {
      const r = Math.floor(idx / n), c = idx % n;
      const wx = -WORLD_HALF + (c + 0.5) * GRID_RES;
      const wy = -WORLD_HALF + (r + 0.5) * GRID_RES;
      const [sx, sy] = w2s(wx, wy);
      mapCacheCtx.fillRect(sx - cs / 2, sy - cs / 2, cs, cs);
    }
    mapCacheCtx.fillStyle = COL.occ;
    for (const idx of exported.occ) {
      const r = Math.floor(idx / n), c = idx % n;
      const wx = -WORLD_HALF + (c + 0.5) * GRID_RES;
      const wy = -WORLD_HALF + (r + 0.5) * GRID_RES;
      const [sx, sy] = w2s(wx, wy);
      mapCacheCtx.fillRect(sx - cs / 2, sy - cs / 2, cs, cs);
    }
    S.mapDirty = false;
  }

  // ═══════════════════════════════════════════════════════════════════════
  // DRAWING
  // ═══════════════════════════════════════════════════════════════════════
  function drawScene() {
    if (S.mapDirty || !mapCacheCanvas.width) redrawMapCache();
    ctx.drawImage(mapCacheCanvas, 0, 0);

    // range rings
    ctx.strokeStyle = COL.grid; ctx.lineWidth = 1;
    for (let r = 2; r <= Math.floor(S.maxRange); r += 2) {
      ctx.beginPath();
      ctx.arc(scene.scx, scene.scy, r * scene.scale, 0, Math.PI * 2);
      ctx.stroke();
    }

    // room resize handles
    if (S.tool === "RESIZE_ROOM") {
      for (const [hx, hy] of roomHandles()) {
        const [sx, sy] = w2s(hx, hy);
        ctx.fillStyle = COL.warn;
        ctx.beginPath(); ctx.arc(sx, sy, 8, 0, Math.PI * 2); ctx.fill();
        ctx.strokeStyle = "#fff"; ctx.lineWidth = 2; ctx.stroke();
      }
    }

    // custom walls
    ctx.lineWidth = 2;
    for (const [x1, y1, x2, y2] of S.customWalls) {
      ctx.strokeStyle = S.tool === "DELETE_OBJ" ? COL.danger : COL.wall;
      const [a, b] = w2s(x1, y1), [c, d] = w2s(x2, y2);
      ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke();
    }
    // room walls
    ctx.strokeStyle = COL.roomWall; ctx.lineWidth = 3;
    const rw = S.roomW, rh = S.roomH;
    const roomEdges = [[-rw, -rh, rw, -rh], [rw, -rh, rw, rh], [rw, rh, -rw, rh], [-rw, rh, -rw, -rh]];
    for (const [x1, y1, x2, y2] of roomEdges) {
      const [a, b] = w2s(x1, y1), [c, d] = w2s(x2, y2);
      ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke();
    }

    // wall preview
    if (S.tool === "ADD_WALL" && S.wallStart && S.wallPreview) {
      ctx.strokeStyle = COL.build; ctx.lineWidth = 2;
      const [a, b] = w2s(...S.wallStart), [c, d] = w2s(...S.wallPreview);
      ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke();
    }

    // path
    if (S.path.length > 1) {
      ctx.strokeStyle = COL.path; ctx.lineWidth = 2;
      ctx.beginPath();
      S.path.forEach((p, i) => {
        const [sx, sy] = w2s(p[0], p[1]);
        if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
      });
      ctx.stroke();
      ctx.fillStyle = COL.path;
      for (const p of S.path) {
        const [sx, sy] = w2s(p[0], p[1]);
        ctx.beginPath(); ctx.arc(sx, sy, 3, 0, Math.PI * 2); ctx.fill();
      }
    }

    // Bug2 M-line overlay
    if (S.bug2Mode && S.goal && S.bug2MLineStart) {
      const [mx1, my1] = S.bug2MLineStart;
      const [mx2, my2] = S.goal;
      const [sx1, sy1] = w2s(mx1, my1);
      const [sx2, sy2] = w2s(mx2, my2);
      ctx.save();
      ctx.strokeStyle = "rgba(167,139,250,0.5)"; ctx.lineWidth = 1.5; ctx.setLineDash([6, 5]);
      ctx.beginPath(); ctx.moveTo(sx1, sy1); ctx.lineTo(sx2, sy2); ctx.stroke();
      ctx.setLineDash([]); ctx.restore();
      // Hit point marker
      if (S.bug2HitPt) {
        const [hpx, hpy] = w2s(...S.bug2HitPt);
        ctx.save(); ctx.fillStyle = "#f5a623"; ctx.strokeStyle = "#000"; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.arc(hpx, hpy, 5, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
        ctx.fillStyle = "#f5a623"; ctx.font = "10px JetBrains Mono"; ctx.textAlign = "center";
        ctx.fillText("H", hpx, hpy - 8);
        ctx.restore();
      }
    }
    // goal marker
    if (S.goal) {
      const [gx, gy] = w2s(...S.goal);
      ctx.strokeStyle = S.goalUnreachable ? COL.danger : COL.goal;
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(gx, gy, 10, 0, Math.PI * 2); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(gx - 10, gy); ctx.lineTo(gx + 10, gy); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(gx, gy - 10); ctx.lineTo(gx, gy + 10); ctx.stroke();
      if (S.goalUnreachable) {
        const alpha = 0.4 + 0.3 * Math.sin(performance.now() / 250);
        ctx.fillStyle = `rgba(226,75,74,${alpha})`;
        ctx.fillRect(gx - 60, gy + 14, 120, 20);
        ctx.fillStyle = COL.danger;
        ctx.font = "11px JetBrains Mono"; ctx.textAlign = "center";
        ctx.fillText("UNREACHABLE", gx, gy + 27);
      }
    }

    // FOV cone + rays
    const half = S.fov / 2.0;
    const [rx0, ry0] = w2s(S.rx, S.ry);
    if (S.fov < 360) {
      ctx.strokeStyle = "rgba(55,134,221,0.25)"; ctx.lineWidth = 1;
      for (const side of [-1, 1]) {
        const edgeAng = (S.rtheta + side * half) % 360;
        const erad = edgeAng * Math.PI / 180;
        const ex = S.rx + Math.cos(erad) * S.maxRange;
        const ey = S.ry + Math.sin(erad) * S.maxRange;
        const [a, b] = w2s(ex, ey);
        ctx.beginPath(); ctx.moveTo(rx0, ry0); ctx.lineTo(a, b); ctx.stroke();
      }
    }
    ctx.strokeStyle = "rgba(55,134,221,0.07)"; ctx.lineWidth = 1;
    for (let i = 0; i < 360; i++) {
      const d = S.distances[i];
      if (d === null) continue;
      const local = (((i - S.rtheta + 540) % 360) - 180);
      if (Math.abs(local) > half + 1) continue;
      const rad = i * Math.PI / 180;
      const hx = S.rx + Math.cos(rad) * d, hy = S.ry + Math.sin(rad) * d;
      const [a, b] = w2s(hx, hy);
      ctx.beginPath(); ctx.moveTo(rx0, ry0); ctx.lineTo(a, b); ctx.stroke();
    }
    // hit points
    ctx.fillStyle = COL.hit;
    for (let i = 0; i < 360; i++) {
      const d = S.distances[i];
      if (d === null) continue;
      const local = (((i - S.rtheta + 540) % 360) - 180);
      if (Math.abs(local) > half + 1) continue;
      const rad = i * Math.PI / 180;
      const hx = S.rx + Math.cos(rad) * d, hy = S.ry + Math.sin(rad) * d;
      const [a, b] = w2s(hx, hy);
      ctx.beginPath(); ctx.arc(a, b, 2, 0, Math.PI * 2); ctx.fill();
    }

    // objects
    for (const o of S.objects) drawObject(o);

    // match ghost
    if (S.matchResult && S.matchResult.matched) {
      const [gx, gy] = w2s(S.matchResult.x, S.matchResult.y);
      ctx.strokeStyle = COL.match; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(gx, gy, 16, 0, Math.PI * 2); ctx.stroke();
      ctx.strokeRect(gx - 40, gy + 18, 80, 20);
      ctx.fillStyle = COL.match; ctx.font = "12px JetBrains Mono"; ctx.textAlign = "center";
      ctx.fillText(`ACCEPT ${(S.matchResult.score * 100).toFixed(0)}%`, gx, gy + 32);
    }

    drawProximityAndRobot();

    // calib arc
    if (S.calibMode) {
      const [rx, ry] = w2s(S.rx, S.ry);
      const prog = S.calibScans.length / 180;
      ctx.strokeStyle = COL.calib; ctx.lineWidth = 3;
      ctx.beginPath(); ctx.arc(rx, ry, 20, -Math.PI / 2, -Math.PI / 2 + prog * Math.PI * 2); ctx.stroke();
    }

    // recording dot
    if (S.recording && Math.floor(performance.now() / 500) % 2 === 0) {
      ctx.fillStyle = COL.danger;
      ctx.beginPath(); ctx.arc(canvas.width - 18, 18, 7, 0, Math.PI * 2); ctx.fill();
      ctx.font = "11px JetBrains Mono"; ctx.textAlign = "right";
      ctx.fillText("REC", canvas.width - 30, 22);
    }
  }

  function drawObject(o) {
    const sel = S.selected === o.id;
    const col = sel ? "#ffa03c" : o.color;
    if (o.kind === "circle") {
      const [px, py] = w2s(o.x, o.y);
      const pr = Math.max(4, o.r * scene.scale);
      ctx.fillStyle = hexToRgba(o.color, 0.2);
      ctx.beginPath(); ctx.arc(px, py, pr, 0, Math.PI * 2); ctx.fill();
      ctx.strokeStyle = col; ctx.lineWidth = sel ? 3 : 2;
      ctx.beginPath(); ctx.arc(px, py, pr, 0, Math.PI * 2); ctx.stroke();
      if (sel) { ctx.strokeStyle = "#fff"; ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(px, py, pr + 4, 0, Math.PI * 2); ctx.stroke(); }
    } else {
      const tl = w2s(o.x - o.w / 2, o.y + o.h / 2);
      const pw = Math.max(4, o.w * scene.scale), ph = Math.max(4, o.h * scene.scale);
      ctx.fillStyle = hexToRgba(o.color, 0.2);
      ctx.fillRect(tl[0], tl[1], pw, ph);
      ctx.strokeStyle = col; ctx.lineWidth = sel ? 3 : 2;
      ctx.strokeRect(tl[0], tl[1], pw, ph);
      if (sel) { ctx.strokeStyle = "#fff"; ctx.lineWidth = 1; ctx.strokeRect(tl[0] - 4, tl[1] - 4, pw + 8, ph + 8); }
    }
  }
  function hexToRgba(hex, a) {
    const v = parseInt(hex.slice(1), 16);
    return `rgba(${(v >> 16) & 255},${(v >> 8) & 255},${v & 255},${a})`;
  }

  function drawProximityAndRobot() {
    const [rx, ry] = w2s(S.rx, S.ry);
    const rr = Math.max(6, S.robotR * scene.scale);
    const fovHalf = S.fov / 2.0;
    const hr = S.rtheta * Math.PI / 180;
    const cf = S.closestFwd;

    let zoneCol = COL.zoneGreen;
    if (cf !== null && cf <= S.proxRed) zoneCol = COL.zoneRed;
    else if (cf !== null && cf <= S.proxYellow) zoneCol = COL.zoneYellow;
    else if (cf !== null && cf <= S.proxGreen) zoneCol = COL.zoneGreen;

    function drawZoneArc(rM, color, alpha) {
      if (rM <= 0) return;
      const rPx = rM * scene.scale;
      if (rPx < 2) return;
      ctx.fillStyle = hexToRgba(color, alpha);
      if (fovHalf >= 180) {
        ctx.beginPath(); ctx.arc(rx, ry, rPx, 0, Math.PI * 2); ctx.fill();
      } else {
        const startAng = -(hr + fovHalf * Math.PI / 180);
        const endAng = -(hr - fovHalf * Math.PI / 180);
        ctx.beginPath(); ctx.moveTo(rx, ry);
        ctx.arc(rx, ry, rPx, startAng, endAng);
        ctx.closePath(); ctx.fill();
      }
    }
    drawZoneArc(S.proxGreen, COL.zoneGreen, 0.07);
    drawZoneArc(S.proxYellow, COL.zoneYellow, 0.11);
    drawZoneArc(S.proxRed, COL.zoneRed, 0.18);

    ctx.lineWidth = 1;
    for (const [rM, col] of [[S.proxGreen, COL.zoneGreen], [S.proxYellow, COL.zoneYellow], [S.proxRed, COL.zoneRed]]) {
      const rPx = rM * scene.scale;
      if (rPx > 0) { ctx.strokeStyle = col; ctx.beginPath(); ctx.arc(rx, ry, rPx, 0, Math.PI * 2); ctx.stroke(); }
    }

    // distance bar
    if (cf !== null) {
      const barW = 120, barH = 10;
      const bx = rx - barW / 2, by = ry - rr - 28;
      ctx.fillStyle = "#141412"; roundRect(bx, by, barW, barH, 3); ctx.fill();
      const frac = Math.max(0, Math.min(1, cf / Math.max(0.1, S.proxGreen)));
      let fillCol = COL.zoneGreen;
      if (cf <= S.proxRed) fillCol = COL.zoneRed; else if (cf <= S.proxYellow) fillCol = COL.zoneYellow;
      ctx.fillStyle = fillCol; roundRect(bx, by, frac * barW, barH, 3); ctx.fill();
      ctx.strokeStyle = "#64645a"; ctx.lineWidth = 1; roundRect(bx, by, barW, barH, 3); ctx.stroke();
      ctx.fillStyle = fillCol; ctx.font = "10px JetBrains Mono"; ctx.textAlign = "center";
      ctx.fillText(cf.toFixed(2) + "m", rx, by - 4);
      let zlbl = "● SAFE";
      if (cf <= S.proxRed) zlbl = "● RED ZONE — STOP"; else if (cf <= S.proxYellow) zlbl = "● WARN ZONE";
      ctx.fillText(zlbl, rx, by + barH + 11);
    }

    // robot body
    ctx.strokeStyle = zoneCol; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(rx, ry, rr + 3, 0, Math.PI * 2); ctx.stroke();
    ctx.fillStyle = COL.robot;
    ctx.beginPath(); ctx.arc(rx, ry, rr, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = COL.robotB; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(rx, ry, rr, 0, Math.PI * 2); ctx.stroke();
    const ex = rx + Math.cos(hr) * (rr + 10), ey = ry - Math.sin(hr) * (rr + 10);
    ctx.strokeStyle = "#ffdc64"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(ex, ey); ctx.stroke();
    ctx.fillStyle = "#ffdc64"; ctx.beginPath(); ctx.arc(ex, ey, 3, 0, Math.PI * 2); ctx.fill();
  }

  function roundRect(x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
  }

  function drawDistanceMatrix() {
    const w = matrixCanvas.width, h = matrixCanvas.height;
    mctx.clearRect(0, 0, w, h);
    const cell = w / 360;
    for (let i = 0; i < 360; i++) {
      const d = S.distances[i];
      let col;
      if (d === null) { col = "#28281f"; }
      else {
        const t = 1 - d / S.maxRange;
        col = lerpColor("#10100e", "#1d9e75", Math.max(0, Math.min(1, t)));
      }
      mctx.fillStyle = col;
      mctx.fillRect(Math.floor(i * cell), 0, Math.ceil(cell) + 1, h);
    }
  }
  function lerpColor(c1, c2, t) {
    const a = parseInt(c1.slice(1), 16), b = parseInt(c2.slice(1), 16);
    const ar = (a >> 16) & 255, ag = (a >> 8) & 255, ab = a & 255;
    const br = (b >> 16) & 255, bg = (b >> 8) & 255, bb = b & 255;
    const r = Math.round(ar + t * (br - ar)), g = Math.round(ag + t * (bg - ag)), bl = Math.round(ab + t * (bb - ab));
    return `rgb(${r},${g},${bl})`;
  }

  // ═══════════════════════════════════════════════════════════════════════
  // STATS / SIDE PANEL
  // ═══════════════════════════════════════════════════════════════════════
  const statGrid = document.getElementById("statGrid");
  const matchBox = document.getElementById("matchBox");
  const tbStatus = document.getElementById("tbStatus");

  function updateStatsPanel() {
    const valid = S.distances.filter(d => d !== null);
    const zone = S.closestFwd === null ? "—" :
      S.closestFwd <= S.proxRed ? "RED" : S.closestFwd <= S.proxYellow ? "WARN" : "SAFE";
    const stats = [
      ["Scan #", S.scanCount], ["Hits", valid.length],
      ["FOV", Math.round(S.fov) + "°"], ["Min", valid.length ? Math.min(...valid).toFixed(3) + "m" : "—"],
      ["Max", valid.length ? Math.max(...valid).toFixed(3) + "m" : "—"],
      ["Robot", `(${S.rx.toFixed(2)},${S.ry.toFixed(2)}) ${S.rtheta.toFixed(0)}°`],
      ["Room", `${(S.roomW * 2).toFixed(1)}×${(S.roomH * 2).toFixed(1)}m`],
      ["Obstacles", S.objects.length], ["Poses", S.savedScans.length],
      ["Path pts", S.path.length],
      ["Fwd dist", S.closestFwd !== null ? S.closestFwd.toFixed(2) + "m" : "—"],
      ["Zone", zone],
      ["Nav mode", S.bug2Mode ? ("Bug2:" + S.bug2State) : "A*"],
      ["Bug2 steps", S.bug2Mode && S.bug2State === "BOUNDARY_FOLLOW" ? S.bug2BoundarySteps : "—"],
    ];
    statGrid.innerHTML = stats.map(([k, v]) =>
      `<div class="stat-line"><span class="k">${k}</span><span class="v">${v}</span></div>`).join("");

    if (S.matchResult && S.matchResult.matched) {
      matchBox.classList.add("show");
      matchBox.innerHTML = `
        <div class="mline">MATCH ${(S.matchResult.score * 100).toFixed(0)}%</div>
        <div class="mline">x=${S.matchResult.x.toFixed(2)} y=${S.matchResult.y.toFixed(2)}</div>
        <div class="mline">ENTER = accept position</div>`;
    } else {
      matchBox.classList.remove("show");
    }
    tbStatus.textContent = S.status;
  }

  // ═══════════════════════════════════════════════════════════════════════
  // TOOLBAR
  // ═══════════════════════════════════════════════════════════════════════
  const TOOL_HINTS = {
    NAVIGATE: "Arrow keys / WASD to move. Q/E rotate.",
    ADD_CIRCLE: "Click scene to place circle.",
    ADD_BOX: "Click scene to place box.",
    ADD_WALL: "Click+drag to draw wall. Right-click to cancel.",
    MOVE_OBJ: "Click object then drag to move.",
    RESIZE_OBJ: "Click object then drag to resize.",
    DELETE_OBJ: "Click object/wall to delete.",
    SET_GOAL: "Click destination — uses A* (or Bug2 if toggled). Bug2 follows M-line + boundary, detects no-path.",
    RESIZE_ROOM: "Drag the orange corner handles to resize room.",
  };
  const TOOL_ACTIVE_COLOR = {
    NAVIGATE: "#1d9e75", ADD_CIRCLE: "#d85a30", ADD_BOX: "#d85a30",
    ADD_WALL: "#d85a30", MOVE_OBJ: "#ef9f27", RESIZE_OBJ: "#ef9f27",
    DELETE_OBJ: "#e24b4a", SET_GOAL: "#ef9f27", RESIZE_ROOM: "#d85a30",
  };

  document.querySelectorAll(".tb-btn[data-tool]").forEach(btn => {
    btn.addEventListener("click", () => setTool(btn.dataset.tool));
  });
  function setTool(tool) {
    S.tool = tool;
    document.querySelectorAll(".tb-btn[data-tool]").forEach(b => {
      const active = b.dataset.tool === tool;
      b.classList.toggle("active", active);
      b.style.background = active ? TOOL_ACTIVE_COLOR[tool] : "";
      b.style.borderColor = active ? TOOL_ACTIVE_COLOR[tool] : "";
    });
    S.goalUnreachable = false;
    S.status = TOOL_HINTS[tool] || "";
  }

  const btnRecord = document.getElementById("btnRecord");
  const btnSave = document.getElementById("btnSave");
  const btnLoad = document.getElementById("btnLoad");
  const btnCalib = document.getElementById("btnCalib");
  const btnClear = document.getElementById("btnClear");
  const mapFileInput = document.getElementById("mapFileInput");

  btnRecord.addEventListener("click", () => {
    S.recording = !S.recording;
    btnRecord.classList.toggle("active", S.recording);
    btnRecord.style.background = S.recording ? "#e24b4a" : "";
    btnRecord.style.borderColor = S.recording ? "#e24b4a" : "";
    S.status = S.recording ? "Recording scan poses…" : "Recording stopped.";
  });

  btnSave.addEventListener("click", saveMapToDownloads);
  btnLoad.addEventListener("click", () => mapFileInput.click());
  mapFileInput.addEventListener("change", handleLoadFile);
  btnCalib.addEventListener("click", startCalibration);
  btnClear.addEventListener("click", () => {
    pyFns.occ_reset(); S.mapDirty = true;
    S.status = "Occupancy map cleared.";
    showToast("Map cleared", "ok");
  });

  // ── SAVE: builds the same JSON shape as the desktop app's _save_map(),
  //          then triggers a real browser download into the Downloads folder.
  function saveMapToDownloads() {
    const data = {
      walls: allWalls(),
      custom_walls: S.customWalls,
      grid: JSON.parse(pyFns.occ_export()),
      scans: S.savedScans,
      room_w: S.roomW, room_h: S.roomH,
    };
    const blob = new Blob([JSON.stringify(data)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    const stamp = new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19);
    a.href = url;
    a.download = `lidar_map_${stamp}.json`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    S.status = `Saved → lidar_map_${stamp}.json`;
    showToast("Map saved to Downloads", "ok");
  }

  // ── LOAD: reads a previously-saved .json back via the browser file picker.
  function handleLoadFile(e) {
    const file = e.target.files[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const data = JSON.parse(reader.result);
        S.customWalls = (data.custom_walls || []).map(w => w.slice());
        pyFns.occ_import(JSON.stringify(data.grid));
        S.savedScans = data.scans || [];
        S.roomW = data.room_w ?? 7.0;
        S.roomH = data.room_h ?? 5.5;
        S.mapDirty = true;
        S.status = `Loaded ← ${file.name} (${S.savedScans.length} poses)`;
        showToast(`Loaded ${file.name}`, "ok");
      } catch (err) {
        console.error(err);
        S.status = "Load failed — invalid map file.";
        showToast("Invalid map file", "err");
      }
      mapFileInput.value = "";
    };
    reader.onerror = () => { showToast("Could not read file", "err"); mapFileInput.value = ""; };
    reader.readAsText(file);
  }

  function showToast(msg, kind) {
    const wrap = document.getElementById("toastWrap");
    const t = document.createElement("div");
    t.className = "toast " + (kind || "");
    t.textContent = msg;
    wrap.appendChild(t);
    requestAnimationFrame(() => t.classList.add("show"));
    setTimeout(() => {
      t.classList.remove("show");
      setTimeout(() => t.remove(), 300);
    }, 3200);
  }

  // ── Calibration / relocalization ──────────────────────────────────────────
  function startCalibration() {
    if (!S.savedScans.length) {
      S.status = "Record + Save a map first!";
      showToast("No saved poses — record + save a map first", "err");
      return;
    }
    S.calibMode = true; S.calibScans = []; S.matchResult = null;
    S.status = "Calibrating: rotating 360°…";
  }
  function calibTick() {
    S.rtheta = (S.rtheta + 2) % 360;
    const dists = new Array(360).fill(null);
    for (let i = 0; i < 360; i++) {
      dists[i] = castSingle(S.rx, S.ry, (S.rtheta + i) % 360);
    }
    S.calibScans.push(dists);
    if (S.calibScans.length >= 180) {
      S.calibMode = false;
      runLocalize();
    }
  }
  function runLocalize() {
    const walls = JSON.stringify(allWalls());
    const result = JSON.parse(pyFns.run_localize_json(
      JSON.stringify(S.calibScans), JSON.stringify(S.savedScans), walls, S.maxRange));
    if (result.matched) {
      S.matchResult = result;
      S.status = `MATCH ${(result.score * 100).toFixed(0)}% at x=${result.x.toFixed(2)} y=${result.y.toFixed(2)}`;
      showToast(`Relocalization match: ${(result.score * 100).toFixed(0)}%`, "ok");
    } else {
      S.matchResult = null;
      S.status = `No match (best=${(result.score * 100).toFixed(0)}%). Drive around more.`;
      showToast("No relocalization match found", "err");
    }
  }

  // ═══════════════════════════════════════════════════════════════════════
  // SLIDERS
  // ═══════════════════════════════════════════════════════════════════════
  function bindSlider(id, valId, stateKey, fmt, onChange) {
    const el = document.getElementById(id);
    const valEl = document.getElementById(valId);
    el.addEventListener("input", () => {
      const v = parseFloat(el.value);
      S[stateKey] = v;
      valEl.textContent = fmt(v);
      if (onChange) onChange(v);
    });
  }
  function f1(v) { return v.toFixed(1); }
  function f0(v) { return Math.round(v).toString(); }
  function f2(v) { return v.toFixed(2); }

  bindSlider("slLidarRange", "vLidarRange", "maxRange", f1);
  bindSlider("slLidarRays", "vLidarRays", "numRays", f0);
  bindSlider("slFov", "vFov", "fov", f0);
  bindSlider("slScanHz", "vScanHz", "scanHz", f0);
  bindSlider("slRobotR", "vRobotR", "robotR", f2);
  bindSlider("slRobotSpeed", "vRobotSpeed", "moveSpeed", f2);
  bindSlider("slObjR", "vObjR", "objR", f2);
  bindSlider("slObjW", "vObjW", "objW", f2);
  bindSlider("slObjH", "vObjH", "objH", f2);
  bindSlider("slProxGreen",  "vProxGreen",  "proxGreen",  v => v.toFixed(1) + " m", v => { const d = document.getElementById("vProxGreenDia");  if(d) d.textContent = (v*2).toFixed(1) + " m"; });
  bindSlider("slProxYellow", "vProxYellow", "proxYellow", v => v.toFixed(1) + " m", v => { const d = document.getElementById("vProxYellowDia"); if(d) d.textContent = (v*2).toFixed(1) + " m"; });
  bindSlider("slProxRed",    "vProxRed",    "proxRed",    v => v.toFixed(1) + " m", v => { const d = document.getElementById("vProxRedDia");    if(d) d.textContent = (v*2).toFixed(1) + " m"; });

  // ═══════════════════════════════════════════════════════════════════════
  // MOUSE / SCENE INTERACTION
  // ═══════════════════════════════════════════════════════════════════════
  function canvasPos(e) {
    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width, scaleY = canvas.height / rect.height;
    return [(e.clientX - rect.left) * scaleX, (e.clientY - rect.top) * scaleY];
  }

  canvas.addEventListener("mousedown", (e) => {
    const [mx, my] = canvasPos(e);
    const [wx, wy] = s2w(mx, my);
    if (e.button === 0) onSceneLeftDown(mx, my, wx, wy);
    else if (e.button === 2) onSceneRightDown(wx, wy);
  });
  canvas.addEventListener("mousemove", (e) => {
    const [mx, my] = canvasPos(e);
    const [wx, wy] = s2w(mx, my);
    onSceneMove(wx, wy);
    canvas.title = `(${wx.toFixed(2)}, ${wy.toFixed(2)})`;
  });
  canvas.addEventListener("mouseup", onSceneUp);
  canvas.addEventListener("contextmenu", (e) => e.preventDefault());

  function onSceneLeftDown(mx, my, wx, wy) {
    if (S.matchResult && S.matchResult.matched) {
      const [gx, gy] = w2s(S.matchResult.x, S.matchResult.y);
      if (mx >= gx - 40 && mx <= gx + 40 && my >= gy + 18 && my <= gy + 38) {
        S.rx = S.matchResult.x; S.ry = S.matchResult.y; S.rtheta = S.matchResult.theta;
        S.matchResult = null; S.status = "Relocated."; return;
      }
    }
    const t = S.tool;
    if (t === "ADD_CIRCLE") { addCircle(wx, wy, S.objR); S.status = `Circle added at (${wx.toFixed(2)},${wy.toFixed(2)})`; }
    else if (t === "ADD_BOX") { addBox(wx, wy, S.objW, S.objH); S.status = `Box added at (${wx.toFixed(2)},${wy.toFixed(2)})`; }
    else if (t === "ADD_WALL") { S.wallStart = [wx, wy]; }
    else if (t === "SET_GOAL") { setGoal(wx, wy); }
    else if (t === "RESIZE_ROOM") {
      const hi = roomHandleHit(wx, wy);
      if (hi !== null) S.resizeHandle = hi;
    } else if (t === "MOVE_OBJ" || t === "RESIZE_OBJ") {
      for (let i = S.objects.length - 1; i >= 0; i--) {
        const o = S.objects[i];
        if (hitObject(o, wx, wy)) {
          S.selected = o.id; S.dragObj = o.id; S.dragOff = [wx - o.x, wy - o.y];
          break;
        }
      }
    } else if (t === "DELETE_OBJ") {
      let deleted = false;
      for (let i = 0; i < S.objects.length; i++) {
        if (hitObject(S.objects[i], wx, wy)) {
          S.objects.splice(i, 1); deleted = true;
          S.selected = null; S.mapDirty = true; S.status = "Object deleted.";
          break;
        }
      }
      if (!deleted) {
        let best = -1, bd = 0.4;
        S.customWalls.forEach((w, i) => {
          const d = distPtSeg(wx, wy, ...w);
          if (d < bd) { bd = d; best = i; }
        });
        if (best >= 0) { S.customWalls.splice(best, 1); S.mapDirty = true; S.status = "Wall deleted."; }
      }
    }
  }
  function onSceneRightDown(wx, wy) {
    if (S.tool === "ADD_WALL" && S.wallStart) { S.wallStart = null; S.wallPreview = null; }
  }
  function onSceneMove(wx, wy) {
    if (S.tool === "ADD_WALL" && S.wallStart) S.wallPreview = [wx, wy];
    else if (S.tool === "MOVE_OBJ" && S.dragObj !== null) {
      const o = S.objects.find(o => o.id === S.dragObj);
      if (o) { o.x = wx - S.dragOff[0]; o.y = wy - S.dragOff[1]; S.mapDirty = true; }
    } else if (S.tool === "RESIZE_OBJ" && S.dragObj !== null) {
      const o = S.objects.find(o => o.id === S.dragObj);
      if (o) {
        const d = Math.hypot(wx - o.x, wy - o.y);
        if (o.kind === "circle") o.r = Math.max(0.1, d);
        else { o.w = Math.max(0.2, Math.abs(wx - o.x) * 2); o.h = Math.max(0.2, Math.abs(wy - o.y) * 2); }
        S.mapDirty = true;
      }
    } else if (S.tool === "RESIZE_ROOM" && S.resizeHandle !== null) {
      if (S.resizeHandle === 0 || S.resizeHandle === 2) S.roomW = Math.max(1.0, Math.min(WORLD_HALF - 0.3, Math.abs(wx)));
      else S.roomH = Math.max(1.0, Math.min(WORLD_HALF - 0.3, Math.abs(wy)));
      S.mapDirty = true;
    }
  }
  function onSceneUp(e) {
    const [mx, my] = canvasPos(e);
    const [wx, wy] = s2w(mx, my);
    if (S.tool === "ADD_WALL" && S.wallStart) {
      if (Math.hypot(wx - S.wallStart[0], wy - S.wallStart[1]) > 0.2) {
        S.customWalls.push([...S.wallStart, wx, wy]);
        S.mapDirty = true; S.status = "Wall added.";
      }
      S.wallStart = null; S.wallPreview = null;
    }
    S.dragObj = null; S.resizeHandle = null;
  }

  function setGoal(wx, wy) {
    S.goal = [wx, wy];
    S.goalUnreachable = false;
    S.bug2NoPath = false;
    const rr = S.robotR;
    if (S.bug2Mode) {
      // Bug2 reactive navigation — no A* grid needed
      initBug2(wx, wy);
      return;
    }
    // A* path planning with 3 retries at shrinking robot radius
    S.path = planPath([S.rx, S.ry], [wx, wy], rr);
    if (S.path.length) { S.pathIdx = 0; S.following = true; S.status = `A* path found: ${S.path.length} waypoints — following…`; return; }
    S.status = "Trying narrower clearance…";
    S.path = planPath([S.rx, S.ry], [wx, wy], rr * 0.5);
    if (S.path.length) { S.pathIdx = 0; S.following = true; S.status = `Narrow path (${S.path.length} pts) — following carefully…`; return; }
    S.path = planPath([S.rx, S.ry], [wx, wy], 0.0);
    if (S.path.length) { S.pathIdx = 0; S.following = true; S.status = `Tight path (${S.path.length} pts) — caution: narrow gap!`; return; }
    // All 3 A* attempts failed
    S.following = false; S.goalUnreachable = true;
    S.status = "UNREACHABLE (A*) — tried 3 clearance levels. No grid path exists.";
  }

  // ── BUG2 ALGORITHM ──────────────────────────────────────────────────────
  // Classic Bug2: robot moves along M-line (start→goal), when it hits an
  // obstacle it follows the boundary until it crosses the M-line again at
  // a point closer to the goal. Detects no-path by returning to hit point.

  function initBug2(gx, gy) {
    S.bug2State = "MOVE_TO_GOAL";
    S.bug2HitPt = null;
    S.bug2HitDistToGoal = null;
    S.bug2BoundarySteps = 0;
    S.bug2NoPath = false;
    // M-line angle: fixed direction from current position to goal
    S.bug2MLineAngle = Math.atan2(gy - S.ry, gx - S.rx) * 180 / Math.PI;
    S.bug2MLineStart = [S.rx, S.ry];
    S.following = true;
    S.status = "Bug2: moving along M-line toward goal…";
  }

  function distToGoal(x, y) {
    if (!S.goal) return 999;
    return Math.hypot(S.goal[0] - x, S.goal[1] - y);
  }

  // Point-to-line distance: is (px,py) on or close to the m-line?
  function onMLine(px, py, tol) {
    if (!S.bug2MLineStart || !S.goal) return false;
    const [x1, y1] = S.bug2MLineStart;
    const [x2, y2] = S.goal;
    const dx = x2 - x1, dy = y2 - y1;
    const len = Math.hypot(dx, dy);
    if (len < 0.01) return false;
    // Perpendicular distance from point to line
    const dist = Math.abs((py - y1) * dx - (px - x1) * dy) / len;
    // Also check that we're between start and goal (not behind start)
    const t = ((px - x1) * dx + (py - y1) * dy) / (len * len);
    return dist < tol && t > 0.1 && t < 1.0;
  }

  function stepBug2() {
    if (!S.goal || !S.following || S.goalUnreachable || S.bug2NoPath) return;
    const step = S.moveSpeed * 2;
    const safeD = S.robotR * 1.5;
    const dg = distToGoal(S.rx, S.ry);

    // ── Reached goal? ──
    if (dg < S.robotR + 0.2) {
      S.following = false; S.bug2State = "IDLE";
      S.status = "Bug2: Goal reached!"; return;
    }

    if (S.bug2State === "MOVE_TO_GOAL") {
      // Aim directly along M-line
      const angToGoal = Math.atan2(S.goal[1] - S.ry, S.goal[0] - S.rx) * 180 / Math.PI;
      const fwdD = castSingle(S.rx, S.ry, angToGoal);
      S.rtheta = ((angToGoal % 360) + 360) % 360;

      if (fwdD !== null && fwdD < safeD) {
        // Hit an obstacle — switch to boundary follow
        S.bug2State = "BOUNDARY_FOLLOW";
        S.bug2HitPt = [S.rx, S.ry];
        S.bug2HitDistToGoal = dg;
        S.bug2BoundarySteps = 0;
        S.status = "Bug2: obstacle hit — following boundary…";
        return;
      }
      // Move along M-line
      const moveStep = Math.min(step, dg - S.robotR * 0.5);
      S.rx += Math.cos(angToGoal * Math.PI / 180) * Math.max(0.01, moveStep);
      S.ry += Math.sin(angToGoal * Math.PI / 180) * Math.max(0.01, moveStep);

    } else if (S.bug2State === "BOUNDARY_FOLLOW") {
      S.bug2BoundarySteps++;

      // No-path detection: came back to hit point without finding M-line crossing
      if (S.bug2BoundarySteps > S.bug2MaxBoundarySteps) {
        S.following = false; S.bug2NoPath = true; S.goalUnreachable = true;
        S.bug2State = "IDLE";
        S.status = "Bug2: NO PATH — boundary loop completed without M-line crossing. Goal is truly unreachable.";
        return;
      }

      // Also detect returning to hit point (tighter check after first 50 steps)
      if (S.bug2HitPt && S.bug2BoundarySteps > 50) {
        const dh = Math.hypot(S.rx - S.bug2HitPt[0], S.ry - S.bug2HitPt[1]);
        if (dh < S.robotR * 0.8) {
          S.following = false; S.bug2NoPath = true; S.goalUnreachable = true;
          S.bug2State = "IDLE";
          S.status = "Bug2: NO PATH — returned to hit point. Obstacle completely surrounds goal.";
          return;
        }
      }

      // Check if we're back on the M-line at a point CLOSER to goal than the hit point
      const mLineTol = S.robotR * 0.8;
      if (S.bug2BoundarySteps > 30 && onMLine(S.rx, S.ry, mLineTol)) {
        const dNow = distToGoal(S.rx, S.ry);
        if (dNow < S.bug2HitDistToGoal - 0.1) {
          // Crossed M-line closer to goal — resume move-to-goal
          S.bug2State = "MOVE_TO_GOAL";
          S.bug2HitPt = null;
          S.status = "Bug2: back on M-line — resuming toward goal…";
          return;
        }
      }

      // Wall-following: try to keep obstacle on the right side.
      // Scan in 8 directions relative to current heading, pick best direction
      // that stays close to wall but doesn't collide.
      const angles = [0, 45, 90, -45, 135, -90, -135, 180];
      let bestAng = null, bestPriority = -1;

      for (const relAng of angles) {
        const absAng = ((S.rtheta + relAng) % 360 + 360) % 360;
        const d = castSingle(S.rx, S.ry, absAng);
        const clear = (d === null || d > safeD);
        // Priority: prefer directions that are clear AND closer to goal
        if (clear) {
          const nx = S.rx + Math.cos(absAng * Math.PI / 180) * step;
          const ny = S.ry + Math.sin(absAng * Math.PI / 180) * step;
          const dToGoal = distToGoal(nx, ny);
          // Score: prefer right-of-current (wall-following) over going backward
          const turnPenalty = Math.abs(relAng) / 180.0;
          const goalBonus = S.bug2HitDistToGoal ? (1 - dToGoal / S.bug2HitDistToGoal) * 0.3 : 0;
          const score = (1 - turnPenalty * 0.5) + goalBonus;
          if (score > bestPriority) { bestPriority = score; bestAng = absAng; }
        }
      }

      if (bestAng === null) {
        // Completely stuck — back up slightly
        const backAng = (S.rtheta + 180) % 360;
        S.rx += Math.cos(backAng * Math.PI / 180) * step * 0.5;
        S.ry += Math.sin(backAng * Math.PI / 180) * step * 0.5;
        S.status = "Bug2: stuck in boundary — backing up…";
        return;
      }
      S.rtheta = bestAng;
      S.rx += Math.cos(bestAng * Math.PI / 180) * step;
      S.ry += Math.sin(bestAng * Math.PI / 180) * step;
    }
    // Clamp to room
    const br = S.robotR;
    S.rx = Math.max(-S.roomW + br, Math.min(S.roomW - br, S.rx));
    S.ry = Math.max(-S.roomH + br, Math.min(S.roomH - br, S.ry));
  }

  function followPath() {
    if (!S.following || !S.path.length) return;
    if (S.pathIdx >= S.path.length) { S.following = false; S.status = "Goal reached!"; return; }
    const [tx, ty] = S.path[S.pathIdx];
    const d = Math.hypot(tx - S.rx, ty - S.ry);
    if (d < 0.15) { S.pathIdx++; return; }
    const ang = Math.atan2(ty - S.ry, tx - S.rx) * 180 / Math.PI;
    const moveAng = ((ang % 360) + 360) % 360;

    const safetyDist = S.robotR * 2.5;
    const fwdD = castSingle(S.rx, S.ry, moveAng);
    if (fwdD !== null && fwdD < safetyDist) {
      S.path = planPath([S.rx, S.ry], S.goal, S.robotR * 0.4);
      if (S.path.length) { S.pathIdx = 0; S.status = "Replanning around obstacle…"; }
      else { S.following = false; S.goalUnreachable = true; S.status = "A* replanning failed — path blocked. Try Bug2 mode or move obstacle."; }
      return;
    }
    S.rtheta = moveAng;
    const spd = Math.min(d, S.moveSpeed * 2);
    S.rx += Math.cos(moveAng * Math.PI / 180) * spd;
    S.ry += Math.sin(moveAng * Math.PI / 180) * spd;
  }

  // ═══════════════════════════════════════════════════════════════════════
  // KEYBOARD MOVEMENT
  // ═══════════════════════════════════════════════════════════════════════
  const keys = {};
  window.addEventListener("keydown", (e) => {
    keys[e.key.toLowerCase()] = true;
    if (e.key === "Enter" && S.matchResult && S.matchResult.matched) {
      S.rx = S.matchResult.x; S.ry = S.matchResult.y; S.rtheta = S.matchResult.theta;
      S.matchResult = null; S.status = "Relocated.";
    }
  });
  window.addEventListener("keyup", (e) => { keys[e.key.toLowerCase()] = false; });

  function handleKeyboardMovement() {
    if (S.tool !== "NAVIGATE" || S.following) return;
    const spd = S.moveSpeed, rot = 3.0;
    let moved = false;
    if (keys["arrowup"] || keys["w"]) {
      const fwdD = castSingle(S.rx, S.ry, S.rtheta);
      if (fwdD === null || fwdD > S.proxRed) {
        S.rx += Math.cos(S.rtheta * Math.PI / 180) * spd;
        S.ry += Math.sin(S.rtheta * Math.PI / 180) * spd;
      } else S.status = "RED ZONE — blocked forward movement!";
      moved = true;
    }
    if (keys["arrowdown"] || keys["s"]) {
      const revD = castSingle(S.rx, S.ry, (S.rtheta + 180) % 360);
      if (revD === null || revD > S.proxRed) {
        S.rx -= Math.cos(S.rtheta * Math.PI / 180) * spd;
        S.ry -= Math.sin(S.rtheta * Math.PI / 180) * spd;
      } else S.status = "RED ZONE — blocked backward movement!";
      moved = true;
    }
    if (keys["arrowleft"] || keys["a"]) { S.rtheta = (S.rtheta + rot) % 360; moved = true; }
    if (keys["arrowright"] || keys["d"]) { S.rtheta = (S.rtheta - rot + 360) % 360; moved = true; }
    if (keys["q"]) { S.rtheta = (S.rtheta + rot) % 360; moved = true; }
    if (keys["e"]) { S.rtheta = (S.rtheta - rot + 360) % 360; moved = true; }
    if (moved) {
      const br = S.robotR;
      S.rx = Math.max(-S.roomW + br, Math.min(S.roomW - br, S.rx));
      S.ry = Math.max(-S.roomH + br, Math.min(S.roomH - br, S.ry));
      S.following = false;
    }
  }

  // ═══════════════════════════════════════════════════════════════════════
  // GAME LOOP
  // ═══════════════════════════════════════════════════════════════════════
  function resizeCanvas() {
    const colEl = canvas.parentElement;
    const w = colEl.clientWidth;
    if (w <= 0) { requestAnimationFrame(resizeCanvas); return; }
    const h = Math.round(w * (812 / 860)); // keep ~same aspect as desktop scene
    canvas.width = w; canvas.height = h;
    computeScene();
    S.mapDirty = true;
  }

  let lastFrame = performance.now();
  function loop(now) {
    const dt = now - lastFrame; lastFrame = now;

    handleKeyboardMovement();
    if (S.calibMode) calibTick();
    if (S.following) { if (S.bug2Mode) { stepBug2(); } else { followPath(); } }

    if ((now - S.lastScanT) >= 1000 / S.scanHz) {
      doScan();
      S.lastScanT = now;
    }

    drawScene();
    drawDistanceMatrix();
    updateStatsPanel();

    requestAnimationFrame(loop);
  }

  // ═══════════════════════════════════════════════════════════════════════
  // BOOT
  // ═══════════════════════════════════════════════════════════════════════
  const loadingBanner = document.getElementById("loading-banner");
  const loadingText = document.getElementById("loading-text");
  const simShell = document.getElementById("simShell");

  async function boot() {
    resolveColors();
    defaultScene();
  
  // Bug2 toggle button
  const btnBug2 = document.getElementById("btnBug2");
  if (btnBug2) {
    btnBug2.addEventListener("click", () => {
      S.bug2Mode = !S.bug2Mode;
      S.bug2State = "IDLE"; S.following = false;
      S.bug2NoPath = false; S.goalUnreachable = false;
      btnBug2.classList.toggle("active", S.bug2Mode);
      S.status = S.bug2Mode
        ? "Bug2 mode ON — click SET GOAL to start. Bug2 follows M-line, hugs boundary when blocked, reports if no path."
        : "Bug2 mode OFF — using A* grid pathfinding.";
    });
  }

  setTool("NAVIGATE");
    resizeCanvas();
    simShell.classList.add("ready");

    try {
      pyodide = await loadPyodide();
      loadingText.textContent = "Loading SLAM engine…";
      pyodide.runPython(PY_SIM_SOURCE);
      bindPyFns();
      pyFns.occ_reset();

      loadingBanner.style.display = "none";
      S.status = "Ready";
      requestAnimationFrame(loop);
    } catch (err) {
      console.error(err);
      loadingText.textContent = "Failed to load Python runtime. Check your connection and reload.";
      loadingBanner.classList.add("bad");
      const sp = loadingBanner.querySelector(".spinner");
      if (sp) sp.style.display = "none";
    }
  }

  window.addEventListener("resize", () => { if (pyodide) resizeCanvas(); });
  boot();
})();
