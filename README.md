# Akshay — Robotics & Automation Lab (Portfolio Site)

A personal project showcase site, built as static HTML/CSS/JS and deployable
straight to **GitHub Pages**. Every project gets its own panel on the
homepage; flagship projects get a fully **live, in-browser demo** powered by
[Pyodide](https://pyodide.org) — real Python, running client-side via
WebAssembly, no backend server required.

---

## What's inside

```
portfolio-site/
├── index.html                      ← Homepage: lists all projects (live + upcoming)
└── projects/
    ├── path-planning/
    │   ├── index.html               ← Project page with live demo + INFO tab
    │   ├── algorithms.js            ← Python source (BFS/DFS/Dijkstra/A*/D* Lite/APF)
    │   ├── algo-info.js             ← Algorithm definitions/principles/complexity
    │   └── app.js                   ← Boots Pyodide, renders canvas grid, handles input
    └── lidar-slam/
        ├── index.html               ← Project page with full simulator UI
        ├── sim-core.js              ← Python SLAM engine (raycasting, occupancy grid, A*, calibration)
        └── app.js                   ← Canvas rendering, toolbar, sliders, save/load, game loop
```

---

## Live demos

### Path Planning Algorithms
BFS, DFS, Dijkstra, A* (Manhattan & Euclidean), D* Lite (with live obstacle
replanning demo), and Potential Field — draw walls, move Start/Goal, watch
each algorithm explore the grid with adjustable animation speed. INFO tab
shows definition, working principle, complexity, and SLAM/ADAS use cases
per algorithm.

### LiDAR SLAM Simulator
Full toolbar-driven 2D SLAM simulator, ported from the desktop pygame app
with **no feature compromises**:
- Room editor (drag corner handles to resize)
- Obstacle tools: add circle/box, move, resize, delete, draw custom walls
- LiDAR raycasting with configurable range, ray count, FOV, scan rate
- Live occupancy-grid SLAM mapping (Bresenham-traced free/occupied cells)
- A* navigation with robot-radius inflation, live obstacle-avoidance
  replanning, and proximity safety zones (green/yellow/red)
- Scan recording → **Save Map** downloads a `.json` to your Downloads
  folder → **Load Map** restores it via a file picker
- Calibration/relocalization: rotate 360°, score-match against saved poses

---

## How the live demos work

1. The browser loads **Pyodide** (a full CPython build compiled to
   WebAssembly) from a CDN — happens once per visit, ~5-10 seconds.
2. Each project's `*-core.js` / `algorithms.js` file contains Python source
   ported directly from the desktop scripts — same math, same algorithms,
   exposed as JSON-friendly functions.
3. The `app.js` for each project calls into that Python code via
   `pyodide.runPython(...)` / `pyodide.globals.get(...)`, and renders the
   result on an HTML5 `<canvas>` every frame.
4. Everything (drawing, navigation, mapping, pathfinding) happens entirely
   client-side — nothing is sent to a server.

### Save / Load (LiDAR SLAM)

Because a web page can't write to your Python script's local disk, **Save
Map** uses the browser's native download mechanism — it builds the exact
same JSON structure as the desktop app's `_save_map()`, wraps it in a
`Blob`, and triggers a download (lands in your Downloads folder, like any
other browser download). **Load Map** opens a native file picker; whatever
`.json` you pick (including ones saved by the desktop pygame app) is read
back in and restores the occupancy grid, custom walls, room size, and
recorded scan poses — so calibration/relocalization keeps working with
previously saved sessions.

---

## Running locally before you publish

You can't just double-click `index.html` — browsers block local file access
needed by Pyodide. Serve it with a tiny local server instead:

```bash
cd portfolio-site
python -m http.server 8000
```

Then open **http://localhost:8000** in your browser.

---

## Deploying to GitHub Pages

1. Create a new GitHub repo (or reuse an existing one) — e.g. `akshaykangude.github.io`
   for a root profile site, or any repo name if you want it under a sub-path.
2. Push this folder's contents to the repo:

   ```bash
   cd portfolio-site
   git init
   git add .
   git commit -m "Initial portfolio site"
   git branch -M main
   git remote add origin https://github.com/YOUR-USERNAME/YOUR-REPO.git
   git push -u origin main
   ```

3. On GitHub: go to **Settings → Pages** → under "Build and deployment",
   set **Source: Deploy from a branch**, **Branch: main**, folder **/ (root)**.
4. Save. GitHub will give you a live URL within a minute or two:
   - `https://YOUR-USERNAME.github.io/YOUR-REPO/` (normal repo)
   - `https://YOUR-USERNAME.github.io/` (if the repo is named `YOUR-USERNAME.github.io`)

---

## Adding a new project

1. Update `index.html` — copy one of the `<div class="panel">` blocks under
   `#projects`, change the title/description/tags/links.
2. If it has a live demo: create `projects/your-project-name/` with its own
   `index.html`, link to it from the homepage panel's "Run Live Demo" button.
3. If it's just a GitHub link: just point the "Source" button at the repo —
   no extra folder needed.
4. Move a project out of the "Upcoming" section into "Active Projects" once
   it's ready.

---

## Browser support note

Pyodide requires a modern browser with WebAssembly support (all current
Chrome/Firefox/Edge/Safari versions work fine). First load downloads ~10MB
of runtime, cached by the browser after that.

---

## Author

**Akshay** — Industrial Automation & Robotics Engineer
