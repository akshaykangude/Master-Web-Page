// app.js
// Boots Pyodide, loads the Python algorithm source, renders the grid on
// <canvas>, handles mouse interaction, animates the algorithm event
// stream, and drives the RUN / INFO tabs.

(function () {
  "use strict";

  // ── Grid config ──────────────────────────────────────────────────────────
  const COLS = 34, ROWS = 23;
  const canvas = document.getElementById("gridCanvas");
  const ctx = canvas.getContext("2d");

  let CELL = 18; // sane default so first paint is never zero-sized

  const FREE = 0, WALL = 1, START = 2, GOAL = 3, OPEN = 4, CLOSED = 5, PATH = 6;

  function getCss(varName) {
    return getComputedStyle(document.documentElement).getPropertyValue(varName).trim();
  }
  // Resolved lazily on first draw — guarantees CSS is fully parsed by then.
  let COLORS = null, GRID_LINE = null;
  function resolveColors() {
    COLORS = {
      [FREE]: getCss("--cell-free"), [WALL]: getCss("--cell-wall"),
      [START]: getCss("--cell-start"), [GOAL]: getCss("--cell-goal"),
      [OPEN]: getCss("--cell-open"), [CLOSED]: getCss("--cell-closed"),
      [PATH]: getCss("--cell-path"),
    };
    GRID_LINE = getCss("--grid-line");
  }

  // ── State ────────────────────────────────────────────────────────────────
  let grid = [];
  let start = [2, 2];
  let goal = [ROWS - 3, COLS - 3];
  let running = false;
  let drawing = false;
  let erasing = false;
  let currentAlgo = "bfs";
  let pyodide = null;

  // Animation speed: ms delay between batches. Index via slider 0-4.
  const SPEED_LEVELS = [
    { label: "Slowest",   delayMs: 90, batchFrac: 600 },
    { label: "Slow",      delayMs: 45, batchFrac: 350 },
    { label: "Normal",    delayMs: 22, batchFrac: 220 },
    { label: "Fast",      delayMs: 8,  batchFrac: 90  },
    { label: "Instant",   delayMs: 0,  batchFrac: 1   },
  ];
  let speedIdx = 2;

  // ── Grid init / maze ─────────────────────────────────────────────────────
  function newGrid() {
    grid = Array.from({ length: ROWS }, () => Array(COLS).fill(FREE));
    grid[start[0]][start[1]] = START;
    grid[goal[0]][goal[1]] = GOAL;
  }

  function range(a, b) { const o = []; for (let i = a; i < b; i++) o.push(i); return o; }

  function addDefaultMaze() {
    const segs = [
      range(2, 16).map(c => [5, c]),
      range(6, 26).map(c => [9, c]),
      range(2, 15).map(c => [13, c]),
      range(10, 30).map(c => [17, c]),
      range(7, 16).map(r => [r, 7]),
      range(2, 11).map(r => [r, 17]),
      range(9, 19).map(r => [r, 26]),
    ];
    const gaps = new Set(["5,8", "5,13", "9,11", "9,21", "13,6", "13,12", "17,15", "17,26"]);
    for (const seg of segs) {
      for (const [r, c] of seg) {
        const key = r + "," + c;
        if (gaps.has(key)) continue;
        if ((r === start[0] && c === start[1]) || (r === goal[0] && c === goal[1])) continue;
        if (r >= 0 && r < ROWS && c >= 0 && c < COLS) grid[r][c] = WALL;
      }
    }
  }

  function clearPathCells() {
    for (let r = 0; r < ROWS; r++)
      for (let c = 0; c < COLS; c++)
        if (grid[r][c] === OPEN || grid[r][c] === CLOSED || grid[r][c] === PATH) grid[r][c] = FREE;
    grid[start[0]][start[1]] = START;
    grid[goal[0]][goal[1]] = GOAL;
  }

  // ── Canvas sizing ────────────────────────────────────────────────────────
  // Robust against the container being width:0 at call time (e.g. mid
  // CSS-transition or before fonts/layout settle) — retries on next frame
  // instead of silently locking in CELL=0 forever.
  function resizeCanvas() {
    const stageEl = canvas.parentElement;
    const stageW = stageEl.clientWidth - 32; // minus padding
    if (stageW <= 0) {
      requestAnimationFrame(resizeCanvas);
      return;
    }
    CELL = Math.max(4, Math.floor(stageW / COLS));
    canvas.width = CELL * COLS;
    canvas.height = CELL * ROWS;
    draw();
  }

  // ── Draw ─────────────────────────────────────────────────────────────────
  function draw() {
    if (!COLORS) resolveColors();
    if (canvas.width === 0 || canvas.height === 0) return;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    for (let r = 0; r < ROWS; r++) {
      for (let c = 0; c < COLS; c++) {
        ctx.fillStyle = COLORS[grid[r][c]];
        ctx.fillRect(c * CELL + 1, r * CELL + 1, CELL - 2, CELL - 2);
      }
    }
    ctx.strokeStyle = GRID_LINE;
    ctx.lineWidth = 1;
    for (let r = 0; r <= ROWS; r++) {
      ctx.beginPath(); ctx.moveTo(0, r * CELL + 0.5); ctx.lineTo(COLS * CELL, r * CELL + 0.5); ctx.stroke();
    }
    for (let c = 0; c <= COLS; c++) {
      ctx.beginPath(); ctx.moveTo(c * CELL + 0.5, 0); ctx.lineTo(c * CELL + 0.5, ROWS * CELL); ctx.stroke();
    }
  }

  // ── Mouse handling ──────────────────────────────────────────────────────
  function cellFromEvent(e) {
    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    const x = (e.clientX - rect.left) * scaleX;
    const y = (e.clientY - rect.top) * scaleY;
    return [Math.floor(y / CELL), Math.floor(x / CELL)];
  }
  function inBounds(r, c) { return r >= 0 && r < ROWS && c >= 0 && c < COLS; }

  function paintAt(e) {
    if (running) return;
    const [r, c] = cellFromEvent(e);
    if (!inBounds(r, c)) return;

    if (e.shiftKey) {
      if (grid[r][c] === GOAL) return;
      grid[start[0]][start[1]] = FREE; start = [r, c]; grid[r][c] = START;
    } else if (e.altKey) {
      if (grid[r][c] === START) return;
      grid[goal[0]][goal[1]] = FREE; goal = [r, c]; grid[r][c] = GOAL;
    } else if (drawing) {
      if (grid[r][c] !== START && grid[r][c] !== GOAL) grid[r][c] = WALL;
    } else if (erasing) {
      if (grid[r][c] === WALL) grid[r][c] = FREE;
    }
    draw();
  }

  canvas.addEventListener("mousedown", (e) => {
    if (e.button === 0) { drawing = true; paintAt(e); }
    else if (e.button === 2) { erasing = true; paintAt(e); }
  });
  canvas.addEventListener("mousemove", (e) => { if (drawing || erasing) paintAt(e); });
  window.addEventListener("mouseup", () => { drawing = false; erasing = false; });
  canvas.addEventListener("contextmenu", (e) => e.preventDefault());

  // ── Stats / status UI ────────────────────────────────────────────────────
  const statAlgo = document.getElementById("statAlgo");
  const statExplored = document.getElementById("statExplored");
  const statPath = document.getElementById("statPath");
  const statTime = document.getElementById("statTime");
  const statusPill = document.getElementById("statusPill");
  const runBtn = document.getElementById("runBtn");
  const speedSlider = document.getElementById("speedSlider");
  const speedVal = document.getElementById("speedVal");

  function setStatus(text, kind) {
    statusPill.textContent = text;
    statusPill.className = "status-pill" + (kind ? " " + kind : "");
  }

  speedSlider.addEventListener("input", () => {
    speedIdx = parseInt(speedSlider.value, 10);
    speedVal.textContent = SPEED_LEVELS[speedIdx].label;
  });

  // ── Tabs (RUN / INFO) ────────────────────────────────────────────────────
  document.querySelectorAll(".tabbtn").forEach((btn) => {
    btn.addEventListener("click", () => {
      document.querySelectorAll(".tabbtn").forEach((b) => b.classList.remove("active"));
      document.querySelectorAll(".tabpane").forEach((p) => p.classList.remove("active"));
      btn.classList.add("active");
      document.getElementById("tab-" + btn.dataset.tab).classList.add("active");
    });
  });

  function renderInfo(algoKey) {
    const info = ALGO_INFO[algoKey];
    const container = document.getElementById("infoContent");
    const yn = (b) => `<span class="prop-val ${b ? "yes" : "no"}">${b ? "YES" : "NO"}</span>`;

    container.innerHTML = `
      <div class="info-name" style="color:${info.color}">${info.name}</div>
      <div class="info-full">${info.fullName}</div>
      <span class="info-pill" style="color:${info.color}; border-color:${info.color}; background:${info.color}22;">${info.category}</span>

      <div class="info-section">
        <div class="info-label">Definition</div>
        <div class="info-body">${info.definition}</div>
      </div>

      <div class="info-section">
        <div class="info-label">Working Principle</div>
        <ul class="info-list">${info.principle.map(s => `<li>${s}</li>`).join("")}</ul>
      </div>

      <div class="info-section">
        <div class="info-label">Complexity</div>
        <table class="complexity-table">
          ${info.complexity.map(([k, v, note]) => `<tr><td>${k}</td><td>${v}</td><td>${note}</td></tr>`).join("")}
        </table>
      </div>

      <div class="info-section">
        <div class="info-label">Properties</div>
        <div class="prop-row"><span>Optimal</span> ${yn(info.optimal)}</div>
        <div class="info-body" style="font-size:11.5px; color:var(--text-faint); margin-top:-4px;">${info.optimalNote}</div>
        <div class="prop-row" style="margin-top:6px;"><span>Complete</span> ${yn(info.complete)}</div>
        <div class="info-body" style="font-size:11.5px; color:var(--text-faint); margin-top:-4px;">${info.completeNote}</div>
      </div>

      <div class="info-section">
        <div class="info-label">Pros</div>
        <ul class="info-list pro-list">${info.pros.map(s => `<li>${s}</li>`).join("")}</ul>
      </div>

      <div class="info-section">
        <div class="info-label">Cons</div>
        <ul class="info-list con-list">${info.cons.map(s => `<li>${s}</li>`).join("")}</ul>
      </div>

      <div class="info-section">
        <div class="info-label">SLAM Use</div>
        <div class="info-body">${info.slam}</div>
      </div>

      <div class="info-section">
        <div class="info-label">ADAS Use</div>
        <div class="info-body">${info.adas}</div>
      </div>

      <div class="tip-box"><b>What to watch:</b> ${info.tip}</div>
    `;
  }

  // ── Algorithm selection ──────────────────────────────────────────────────
  document.querySelectorAll(".algo-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
      if (running) return;
      document.querySelectorAll(".algo-btn").forEach((b) => b.classList.remove("active"));
      btn.classList.add("active");
      currentAlgo = btn.dataset.algo;
      statAlgo.textContent = ALGO_INFO[currentAlgo].name;
      setStatus("Ready — press Run", "");
      renderInfo(currentAlgo);
    });
  });

  // ── Run / Clear / Reset ──────────────────────────────────────────────────
  document.getElementById("clearBtn").addEventListener("click", () => {
    if (running) return;
    clearPathCells(); draw();
    statExplored.textContent = "—"; statPath.textContent = "—"; statTime.textContent = "—";
    setStatus("Path cleared", "");
  });

  document.getElementById("resetBtn").addEventListener("click", () => {
    if (running) return;
    start = [2, 2]; goal = [ROWS - 3, COLS - 3];
    newGrid(); addDefaultMaze(); draw();
    statExplored.textContent = "—"; statPath.textContent = "—"; statTime.textContent = "—";
    setStatus("Grid reset", "");
  });

  runBtn.addEventListener("click", runAlgorithm);

  async function runAlgorithm() {
    if (running || !pyodide) return;
    clearPathCells(); draw();
    running = true;
    runBtn.disabled = true;
    setStatus("Running " + ALGO_INFO[currentAlgo].name + "…", "busy");
    statExplored.textContent = "0"; statPath.textContent = "—"; statTime.textContent = "—";

    const t0 = performance.now();
    const gridJson = JSON.stringify(grid.map(row => row.map(v => (v === WALL ? 1 : 0))));
    const startJson = JSON.stringify(start);
    const goalJson = JSON.stringify(goal);

    let eventsJson;
    try {
      const runFn = pyodide.globals.get("run_algorithm");
      eventsJson = runFn(currentAlgo, gridJson, startJson, goalJson);
    } catch (err) {
      console.error(err);
      setStatus("Error — see browser console", "bad");
      running = false; runBtn.disabled = false;
      return;
    }

    const events = JSON.parse(eventsJson);
    animateEvents(events, t0);
  }

  function animateEvents(events, t0) {
    let i = 0;
    let explored = 0, pathLen = 0;
    const level = SPEED_LEVELS[speedIdx];
    const STEP_BATCH = Math.max(1, Math.ceil(events.length / level.batchFrac));
    let lastFrameTime = 0;
    let pauseUntil = 0; // performance.now() timestamp to hold the frame until

    function tick(now) {
      if (now < pauseUntil) {
        requestAnimationFrame(tick);
        return;
      }
      if (now - lastFrameTime < level.delayMs) {
        requestAnimationFrame(tick);
        return;
      }
      lastFrameTime = now;

      const batchEnd = Math.min(i + STEP_BATCH, events.length);
      for (; i < batchEnd; i++) {
        const [type, r, c] = events[i];
        if (type === "open") {
          if (grid[r][c] !== START && grid[r][c] !== GOAL) grid[r][c] = OPEN;
          explored++;
        } else if (type === "close") {
          if (grid[r][c] !== START && grid[r][c] !== GOAL) grid[r][c] = CLOSED;
        } else if (type === "path") {
          if (grid[r][c] !== START && grid[r][c] !== GOAL) grid[r][c] = PATH;
          pathLen++;
        } else if (type === "wall") {
          // D* Lite: a new obstacle is dropped mid-run — show it clearly,
          // then pause briefly so the moment registers before the replan.
          if (grid[r][c] !== START && grid[r][c] !== GOAL) grid[r][c] = WALL;
          draw();
          setStatus("⚠ New obstacle detected — replanning…", "busy");
          pauseUntil = now + 700;
        } else if (type === "clear_path") {
          // Wipe OPEN/CLOSED/PATH (not walls/start/goal) before showing
          // the repaired route, and reset the path counter for it.
          for (let rr = 0; rr < ROWS; rr++) {
            for (let cc = 0; cc < COLS; cc++) {
              if (grid[rr][cc] === OPEN || grid[rr][cc] === CLOSED || grid[rr][cc] === PATH) {
                grid[rr][cc] = FREE;
              }
            }
          }
          grid[start[0]][start[1]] = START;
          grid[goal[0]][goal[1]] = GOAL;
          pathLen = 0;
          statPath.textContent = "—";
        }
      }
      draw();
      statExplored.textContent = explored;
      if (pathLen > 0) statPath.textContent = pathLen;

      if (i < events.length) {
        requestAnimationFrame(tick);
      } else {
        const ms = (performance.now() - t0).toFixed(1);
        statTime.textContent = ms + " ms";
        if (pathLen > 0) {
          setStatus("✓ Path repaired — " + pathLen + " steps", "ok");
        } else {
          setStatus("✗ No path found", "bad");
          statPath.textContent = "none";
        }
        running = false;
        runBtn.disabled = false;
      }
    }
    requestAnimationFrame(tick);
  }

  // ── Boot Pyodide ─────────────────────────────────────────────────────────
  const loadingBanner = document.getElementById("loading-banner");
  const loadingText = document.getElementById("loading-text");
  const app = document.getElementById("app");

  async function boot() {
    // Build the maze + draw an initial frame immediately, BEFORE Pyodide
    // finishes loading, so the grid is never blank while the (slow) WASM
    // runtime boots in the background.
    newGrid();
    addDefaultMaze();
    resizeCanvas();
    app.classList.add("ready");
    renderInfo(currentAlgo);

    try {
      pyodide = await loadPyodide();
      loadingText.textContent = "Loading algorithm code…";
      pyodide.runPython(PY_ALGORITHMS_SOURCE);

      loadingBanner.style.display = "none";
      runBtn.disabled = false;
      setStatus("Ready — press Run", "");
    } catch (err) {
      console.error(err);
      loadingText.textContent = "Failed to load Python runtime. Check your connection and reload the page.";
      loadingBanner.classList.add("bad");
      const spinner = loadingBanner.querySelector(".spinner");
      if (spinner) spinner.style.display = "none";
    }
  }

  window.addEventListener("resize", () => resizeCanvas());
  boot();
})();
