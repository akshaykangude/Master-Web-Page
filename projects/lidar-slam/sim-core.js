// sim-core.js
// Pure-Python simulation engine ported directly from the desktop
// lidar_slam_ui.py — raycasting math, occupancy-grid SLAM mapping,
// A* pathfinding with robot-radius inflation, and pose-matching
// calibration/relocalization. Executed inside Pyodide; the JS/Canvas
// layer (app.js) drives it every frame and reads back JSON state.

const PY_SIM_SOURCE = `
import math, json, heapq

# ─── WORLD CONSTANTS (mirrors lidar_slam_ui.py) ──────────────────────────────
WORLD_HALF = 9.0
GRID_RES   = 0.08
GRID_N     = int(WORLD_HALF*2/GRID_RES)

UNK, FREE_V, OCC_V = 128, 220, 20

# ─── RAY MATH ─────────────────────────────────────────────────────────────────
def ray_seg(ox,oy,dx,dy,x1,y1,x2,y2):
    sx,sy=x2-x1,y2-y1
    d=dx*sy-dy*sx
    if abs(d)<1e-9: return None
    t=((x1-ox)*sy-(y1-oy)*sx)/d
    u=((x1-ox)*dy-(y1-oy)*dx)/d
    return t if t>1e-5 and 0<=u<=1 else None

def ray_circle(ox,oy,dx,dy,cx,cy,r):
    fx,fy=ox-cx,oy-cy
    a=dx*dx+dy*dy; b=2*(fx*dx+fy*dy); c=fx*fx+fy*fy-r*r
    disc=b*b-4*a*c
    if disc<0: return None
    t=(-b-math.sqrt(disc))/(2*a)
    return t if t>1e-5 else None

def ray_box(ox,oy,dx,dy,bx,by,bw,bh):
    segs=[(bx,by,bx+bw,by),(bx+bw,by,bx+bw,by+bh),
          (bx+bw,by+bh,bx,by+bh),(bx,by+bh,bx,by)]
    mt=None
    for s in segs:
        t=ray_seg(ox,oy,dx,dy,*s)
        if t and (mt is None or t<mt): mt=t
    return mt

def dist_pt_seg(px,py,x1,y1,x2,y2):
    dx,dy=x2-x1,y2-y1
    if dx==0 and dy==0: return math.hypot(px-x1,py-y1)
    t=max(0,min(1,((px-x1)*dx+(py-y1)*dy)/(dx*dx+dy*dy)))
    return math.hypot(px-(x1+t*dx),py-(y1+t*dy))

def bresenham(r0,c0,r1,c1):
    pts=[]
    dr,dc=abs(r1-r0),abs(c1-c0)
    sr=1 if r0<r1 else -1; sc=1 if c0<c1 else -1; err=dr-dc
    while True:
        pts.append((r0,c0))
        if r0==r1 and c0==c1: break
        e2=2*err
        if e2>-dc: err-=dc; r0+=sr
        if e2< dr: err+=dr; c0+=sc
    return pts

# ─── OCCUPANCY GRID ─────────────────────────────────────────────────────────
class OccGrid:
    def __init__(self):
        self.g = [[UNK]*GRID_N for _ in range(GRID_N)]

    def w2c(self, wx, wy):
        c = int((wx+WORLD_HALF)/GRID_RES); r = int((wy+WORLD_HALF)/GRID_RES)
        if 0<=r<GRID_N and 0<=c<GRID_N: return (r,c)
        return None

    def c2w(self, r, c):
        return (-WORLD_HALF+(c+0.5)*GRID_RES, -WORLD_HALF+(r+0.5)*GRID_RES)

    def update(self, rx, ry, hits):
        rc = self.w2c(rx, ry)
        if not rc: return
        for hx, hy in hits:
            hc = self.w2c(hx, hy)
            if not hc: continue
            for r, c in bresenham(rc[0], rc[1], hc[0], hc[1]):
                if self.g[r][c] != OCC_V: self.g[r][c] = FREE_V
            self.g[hc[0]][hc[1]] = OCC_V

    def to_compact(self):
        # Sparse encoding: only non-UNK cells, far smaller than dumping the
        # full 225x225 grid every frame to JS.
        free_cells = []
        occ_cells = []
        for r in range(GRID_N):
            row = self.g[r]
            for c in range(GRID_N):
                v = row[c]
                if v == FREE_V: free_cells.append(r*GRID_N+c)
                elif v == OCC_V: occ_cells.append(r*GRID_N+c)
        return {"free": free_cells, "occ": occ_cells, "n": GRID_N}

    def from_compact(self, d):
        n = d["n"]
        self.g = [[UNK]*n for _ in range(n)]
        for idx in d["free"]:
            self.g[idx//n][idx%n] = FREE_V
        for idx in d["occ"]:
            self.g[idx//n][idx%n] = OCC_V

    def clone(self):
        o = OccGrid()
        o.g = [row[:] for row in self.g]
        return o


# ─── A* WITH ROBOT-RADIUS INFLATION ─────────────────────────────────────────
def _inflate_grid(grid, pad_cells):
    n = GRID_N
    occ_mask = [[grid.g[r][c]==OCC_V for c in range(n)] for r in range(n)]
    if pad_cells == 0:
        return occ_mask
    blocked = [[False]*n for _ in range(n)]
    p = pad_cells
    offsets = []
    for dr in range(-p, p+1):
        for dc in range(-p, p+1):
            if math.hypot(dr,dc) <= p+0.5:
                offsets.append((dr,dc))
    for r in range(n):
        for c in range(n):
            if not occ_mask[r][c]: continue
            for dr, dc in offsets:
                nr, nc = r+dr, c+dc
                if 0<=nr<n and 0<=nc<n:
                    blocked[nr][nc] = True
    return blocked


def astar_grid(grid, start_w, goal_w, robot_r):
    sc = grid.w2c(*start_w); gc = grid.w2c(*goal_w)
    if not sc or not gc: return []
    pad = max(0, int(math.ceil(robot_r/GRID_RES)))
    blocked_mask = _inflate_grid(grid, pad)
    n = GRID_N

    def blocked(r,c):
        if not (0<=r<n and 0<=c<n): return True
        return blocked_mask[r][c]

    def h(a,b): return math.hypot(a[0]-b[0], a[1]-b[1])

    open_set = [(h(sc,gc), 0.0, sc)]
    came = {}; gs = {sc: 0.0}
    dirs = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    visited_cap = 40000  # safety cap for browser responsiveness
    visited = 0

    while open_set and visited < visited_cap:
        _, cost, cur = heapq.heappop(open_set)
        visited += 1
        if cur == gc:
            path = []
            node = cur
            while node in came:
                path.append(grid.c2w(*node)); node = came[node]
            path.append(grid.c2w(*sc))
            path.reverse()
            return path
        for dr, dc in dirs:
            nb = (cur[0]+dr, cur[1]+dc)
            if blocked(*nb): continue
            ng = cost + (1.414 if abs(dr)+abs(dc)==2 else 1.0)
            if ng < gs.get(nb, 1e18):
                came[nb] = cur; gs[nb] = ng
                heapq.heappush(open_set, (ng+h(nb,gc), ng, nb))
    return []


# ─── SCENE OBJECT CASTING (circle / box obstacles) ──────────────────────────
def cast_objects(ox, oy, dx, dy, objects):
    """objects: list of dicts {kind, x, y, r} or {kind, x, y, w, h}"""
    mt = None
    for o in objects:
        if o["kind"] == "circle":
            t = ray_circle(ox, oy, dx, dy, o["x"], o["y"], o["r"])
        else:
            t = ray_box(ox, oy, dx, dy, o["x"]-o["w"]/2, o["y"]-o["h"]/2, o["w"], o["h"])
        if t is not None and (mt is None or t < mt):
            mt = t
    return mt


def cast_ray(ox, oy, angle_deg, walls, objects, max_range):
    rad = math.radians(angle_deg)
    dx, dy = math.cos(rad), math.sin(rad)
    mt = max_range
    for x1,y1,x2,y2 in walls:
        t = ray_seg(ox,oy,dx,dy,x1,y1,x2,y2)
        if t is not None and t < mt: mt = t
    ot = cast_objects(ox,oy,dx,dy,objects)
    if ot is not None and ot < mt: mt = ot
    return mt if mt < max_range else None


# ─── FULL 360 SCAN ───────────────────────────────────────────────────────────
def do_scan(rx, ry, rtheta, num_rays, fov, walls, objects, max_range):
    """
    Returns:
      distances: list[360] of float|None  (indexed by absolute angle deg)
      hits: list of (x,y) world points
    """
    half = fov/2.0
    distances = [None]*360
    hits = []
    nr = max(1, int(num_rays))
    for i in range(nr):
        local_ang = -half + (i/(max(1,nr-1)))*fov if nr>1 else 0
        abs_ang = int((rtheta+local_ang) % 360)
        d = cast_ray(rx, ry, (rtheta+local_ang) % 360, walls, objects, max_range)
        distances[abs_ang] = d
        if d is not None:
            rad = math.radians((rtheta+local_ang) % 360)
            hits.append((rx+math.cos(rad)*d, ry+math.sin(rad)*d))
    return distances, hits


# ─── SCAN-MATCH SCORING (for calibration/relocalization) ────────────────────
def score_pose(scan, px, py, ptheta, walls, max_range):
    tol = 0.25; ok = 0; total = 0
    for i in range(0, 360, 4):
        d = scan[i]
        if d is None: continue
        total += 1
        rad = math.radians((ptheta+i) % 360)
        dx, dy = math.cos(rad), math.sin(rad)
        mt = max_range
        for x1,y1,x2,y2 in walls:
            t = ray_seg(px,py,dx,dy,x1,y1,x2,y2)
            if t is not None and t < mt: mt = t
        if mt < max_range and abs(d-mt) < tol: ok += 1
    return ok/total if total else 0.0


def run_localize(calib_scans, saved_scans, walls, max_range):
    best_s = 0.0; best_p = None
    for dists in calib_scans:
        for pose in saved_scans:
            s = score_pose(dists, pose["x"], pose["y"], pose["theta"], walls, max_range)
            if s > best_s:
                best_s = s; best_p = (pose["x"], pose["y"], pose["theta"])
    if best_p and best_s >= 0.80:
        return {"matched": True, "x": best_p[0], "y": best_p[1], "theta": best_p[2], "score": best_s}
    return {"matched": False, "score": best_s}


# ─── JSON-FRIENDLY ENTRY POINTS (called from JS) ─────────────────────────────

_occ = OccGrid()

def occ_reset():
    global _occ
    _occ = OccGrid()

def occ_update(rx, ry, hits_json):
    hits = json.loads(hits_json)
    _occ.update(rx, ry, hits)

def occ_export():
    return json.dumps(_occ.to_compact())

def occ_import(data_json):
    global _occ
    _occ.from_compact(json.loads(data_json))

def run_scan(rx, ry, rtheta, num_rays, fov, walls_json, objects_json, max_range):
    walls = json.loads(walls_json)
    objects = json.loads(objects_json)
    distances, hits = do_scan(rx, ry, rtheta, num_rays, fov, walls, objects, max_range)
    _occ.update(rx, ry, hits)
    return json.dumps({"distances": distances, "hits": hits})

def run_astar(start_json, goal_json, robot_r):
    start = tuple(json.loads(start_json))
    goal = tuple(json.loads(goal_json))
    path = astar_grid(_occ, start, goal, robot_r)
    return json.dumps(path)

def run_score_pose(scan_json, px, py, ptheta, walls_json, max_range):
    scan = json.loads(scan_json)
    walls = json.loads(walls_json)
    return score_pose(scan, px, py, ptheta, walls, max_range)

def run_localize_json(calib_scans_json, saved_scans_json, walls_json, max_range):
    calib_scans = json.loads(calib_scans_json)
    saved_scans = json.loads(saved_scans_json)
    walls = json.loads(walls_json)
    result = run_localize(calib_scans, saved_scans, walls, max_range)
    return json.dumps(result)

def run_cast_single(ox, oy, angle_deg, walls_json, objects_json, max_range):
    walls = json.loads(walls_json)
    objects = json.loads(objects_json)
    d = cast_ray(ox, oy, angle_deg, walls, objects, max_range)
    return json.dumps(d)
`;
