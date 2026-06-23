// algorithms.js
// Pure-Python source for BFS, DFS, Dijkstra, A*, and Potential Field,
// ported directly from the desktop visualizer.py generators in the
// Path-Planning-Algorithms GitHub repo. This string is executed inside
// Pyodide (CPython compiled to WebAssembly) — it is the SAME algorithm
// logic, unmodified in substance, just exposed as JSON-friendly functions
// for the JS/Canvas renderer to call.

const PY_ALGORITHMS_SOURCE = `
import heapq, math, random, json
from collections import deque

D4 = [(-1,0),(1,0),(0,-1),(0,1)]

def is_free(grid, r, c):
    rows = len(grid); cols = len(grid[0])
    return 0 <= r < rows and 0 <= c < cols and grid[r][c] != 1

def backtrack(prev, start, goal):
    path, node = [], goal
    while node in prev:
        path.append(node); node = prev[node]
    if node == start:
        path.append(start); path.reverse(); return path
    return []

def run_bfs(grid, start, goal):
    start = tuple(start); goal = tuple(goal)
    events = []
    q = deque([(start, [start])]); visited = {start}
    while q:
        (r,c), path = q.popleft()
        events.append(("close", r, c))
        if (r,c) == goal:
            for p in path: events.append(("path", p[0], p[1]))
            return events
        for dr, dc in D4:
            nr, nc = r+dr, c+dc
            if is_free(grid, nr, nc) and (nr,nc) not in visited:
                visited.add((nr,nc))
                events.append(("open", nr, nc))
                q.append(((nr,nc), path+[(nr,nc)]))
    events.append(("fail", -1, -1))
    return events

def run_dfs(grid, start, goal):
    start = tuple(start); goal = tuple(goal)
    events = []
    stack = [(start, [start])]; visited = set()
    while stack:
        (r,c), path = stack.pop()
        if (r,c) in visited: continue
        visited.add((r,c))
        events.append(("close", r, c))
        if (r,c) == goal:
            for p in path: events.append(("path", p[0], p[1]))
            return events
        for dr, dc in D4:
            nr, nc = r+dr, c+dc
            if is_free(grid, nr, nc) and (nr,nc) not in visited:
                events.append(("open", nr, nc))
                stack.append(((nr,nc), path+[(nr,nc)]))
    events.append(("fail", -1, -1))
    return events

def run_dijkstra(grid, start, goal):
    start = tuple(start); goal = tuple(goal)
    events = []
    dist = {start: 0}; prev = {}
    heap = [(0, start)]; done = set()
    while heap:
        cost, (r,c) = heapq.heappop(heap)
        if (r,c) in done: continue
        done.add((r,c))
        events.append(("close", r, c))
        if (r,c) == goal:
            for p in backtrack(prev, start, goal): events.append(("path", p[0], p[1]))
            return events
        for dr, dc in D4:
            nr, nc = r+dr, c+dc
            if is_free(grid, nr, nc) and (nr,nc) not in done:
                nc2 = cost + 1
                if nc2 < dist.get((nr,nc), float('inf')):
                    dist[(nr,nc)] = nc2; prev[(nr,nc)] = (r,c)
                    events.append(("open", nr, nc))
                    heapq.heappush(heap, (nc2, (nr,nc)))
    events.append(("fail", -1, -1))
    return events

def h_manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def h_euclidean(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def run_astar(grid, start, goal, heuristic_name, diagonal):
    start = tuple(start); goal = tuple(goal)
    h = h_manhattan if heuristic_name == "manhattan" else h_euclidean
    if diagonal:
        dirs = [(-1,0,1.0),(1,0,1.0),(0,-1,1.0),(0,1,1.0),
                (-1,-1,1.414),(-1,1,1.414),(1,-1,1.414),(1,1,1.414)]
    else:
        dirs = [(-1,0,1.0),(1,0,1.0),(0,-1,1.0),(0,1,1.0)]

    events = []
    g = {start: 0}; prev = {}
    heap = [(h(start, goal), start)]; done = set()
    while heap:
        f, (r,c) = heapq.heappop(heap)
        if (r,c) in done: continue
        done.add((r,c))
        events.append(("close", r, c))
        if (r,c) == goal:
            for p in backtrack(prev, start, goal): events.append(("path", p[0], p[1]))
            return events
        for dr, dc, mc in dirs:
            nr, nc = r+dr, c+dc
            if is_free(grid, nr, nc) and (nr,nc) not in done:
                ng = g[(r,c)] + mc
                if ng < g.get((nr,nc), float('inf')):
                    g[(nr,nc)] = ng; prev[(nr,nc)] = (r,c)
                    events.append(("open", nr, nc))
                    heapq.heappush(heap, (ng + h((nr,nc), goal), (nr,nc)))
    events.append(("fail", -1, -1))
    return events

class DStarLite:
    """D* Lite on a 2-D grid. Ported directly from algorithms/dstar_lite.py."""
    INF = float('inf')

    def __init__(self, grid, start, goal):
        self.grid = [row[:] for row in grid]
        self.rows = len(grid); self.cols = len(grid[0])
        self.start = start; self.goal = goal
        self.robot = start
        self.DIRS = [(-1,0),(1,0),(0,-1),(0,1)]
        self.km = 0
        self.g = {s: self.INF for s in self._all_cells()}
        self.rhs = {s: self.INF for s in self._all_cells()}
        self.rhs[goal] = 0
        self._heap = []
        self._heap_set = {}
        self._push(goal, self._calc_key(goal))

    def _all_cells(self):
        return [(r, c) for r in range(self.rows) for c in range(self.cols)]

    def _heuristic(self, a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    def _calc_key(self, s):
        g_rhs = min(self.g[s], self.rhs[s])
        return (g_rhs + self._heuristic(self.robot, s) + self.km, g_rhs)

    def _push(self, node, key):
        heapq.heappush(self._heap, (key, node))
        self._heap_set[node] = key

    def _pop(self):
        while self._heap:
            key, node = heapq.heappop(self._heap)
            if self._heap_set.get(node) == key:
                del self._heap_set[node]
                return key, node
        return None, None

    def _top_key(self):
        while self._heap:
            key, node = self._heap[0]
            if self._heap_set.get(node) == key:
                return key
            heapq.heappop(self._heap)
        return (self.INF, self.INF)

    def _cost(self, a, b):
        r, c = b
        if self.grid[r][c] == 1:
            return self.INF
        return 1.0

    def _neighbours(self, s):
        r, c = s
        result = []
        for dr, dc in self.DIRS:
            nr, nc = r+dr, c+dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                result.append((nr, nc))
        return result

    def _update_vertex(self, u):
        if u != self.goal:
            self.rhs[u] = min(self._cost(u, s) + self.g[s] for s in self._neighbours(u))
        if u in self._heap_set:
            del self._heap_set[u]
        if self.g[u] != self.rhs[u]:
            self._push(u, self._calc_key(u))

    def compute_shortest_path(self):
        start = self.robot
        while (self._top_key() < self._calc_key(start) or self.rhs[start] != self.g[start]):
            k_old, u = self._pop()
            if u is None:
                break
            k_new = self._calc_key(u)
            if k_old < k_new:
                self._push(u, k_new)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for s in self._neighbours(u):
                    self._update_vertex(s)
            else:
                self.g[u] = self.INF
                self._update_vertex(u)
                for s in self._neighbours(u):
                    self._update_vertex(s)

    def get_path(self):
        path = [self.robot]
        current = self.robot
        visited = {current}
        while current != self.goal:
            best = None; best_cost = self.INF
            for n in self._neighbours(current):
                c = self._cost(current, n) + self.g[n]
                if c < best_cost:
                    best_cost = c; best = n
            if best is None or best in visited:
                return []
            path.append(best); visited.add(best)
            current = best
        return path

    def update_obstacle(self, cell, is_obstacle):
        r, c = cell
        new_val = 1 if is_obstacle else 0
        if self.grid[r][c] == new_val:
            return
        self.grid[r][c] = new_val
        self._update_vertex(cell)
        for n in self._neighbours(cell):
            self._update_vertex(n)


def run_dstar(grid, start, goal):
    start = tuple(start); goal = tuple(goal)
    events = []

    planner = DStarLite(grid, start, goal)
    planner.compute_shortest_path()
    path = planner.get_path()

    if not path:
        events.append(("fail", -1, -1))
        return events

    # Show the search settling (close events) then the initial path
    for cell in path:
        events.append(("close", cell[0], cell[1]))
    for p in path:
        events.append(("path", p[0], p[1]))

    # Mid-path: drop a new obstacle and repair the plan live —
    # this is the entire point of D* Lite vs plain A*.
    if len(path) > 4:
        mid = path[len(path)//2]
        if mid != start and mid != goal:
            events.append(("wall", mid[0], mid[1]))
            planner.update_obstacle(mid, True)
            planner.compute_shortest_path()
            new_path = planner.get_path()

            events.append(("clear_path", -1, -1))

            if new_path:
                for cell in new_path:
                    events.append(("close", cell[0], cell[1]))
                for p in new_path:
                    events.append(("path", p[0], p[1]))
            else:
                events.append(("fail", -1, -1))

    return events


def run_apf(grid, start, goal):
    start = tuple(start); goal = tuple(goal)
    rows = len(grid); cols = len(grid[0])
    obstacles = [(r,c) for r in range(rows) for c in range(cols) if grid[r][c] == 1]

    pos = [float(start[0]), float(start[1])]
    seen = set(); path_cells = []
    K_ATT = 2.0; K_REP = 150.0; D0 = 3.5; STEP = 0.25
    events = []

    random.seed(42)

    for _ in range(6000):
        cr = int(round(pos[0])); cc = int(round(pos[1]))
        cell = (max(0,min(rows-1,cr)), max(0,min(cols-1,cc)))
        if cell not in seen:
            seen.add(cell); path_cells.append(cell)
            if cell == goal:
                events.append(("close", cell[0], cell[1]))
                break
            events.append(("open", cell[0], cell[1]))

        dr_ = goal[0]-pos[0]; dc_ = goal[1]-pos[1]
        d_goal = math.sqrt(dr_**2 + dc_**2)
        if d_goal < 0.4: break
        fr = K_ATT*dr_; fc = K_ATT*dc_

        for (or_, oc) in obstacles:
            dr = pos[0]-or_; dc = pos[1]-oc
            d = math.sqrt(dr**2 + dc**2)
            if 0 < d < D0:
                coeff = K_REP*(1.0/d - 1.0/D0)/(d*d)
                fr += coeff*(dr/d); fc += coeff*(dc/d)

        mag = math.sqrt(fr**2 + fc**2)
        if mag < 1e-4:
            angle = random.uniform(0, 2*math.pi)
            fr = math.cos(angle); fc = math.sin(angle); mag = 1.0

        pos[0] = max(0, min(rows-1, pos[0] + STEP*fr/mag))
        pos[1] = max(0, min(cols-1, pos[1] + STEP*fc/mag))

    for p in path_cells:
        events.append(("path", p[0], p[1]))
    return events


def run_algorithm(algo_name, grid_json, start_json, goal_json):
    grid = json.loads(grid_json)
    start = json.loads(start_json)
    goal = json.loads(goal_json)

    if algo_name == "bfs":
        events = run_bfs(grid, start, goal)
    elif algo_name == "dfs":
        events = run_dfs(grid, start, goal)
    elif algo_name == "dijkstra":
        events = run_dijkstra(grid, start, goal)
    elif algo_name == "astar_m":
        events = run_astar(grid, start, goal, "manhattan", False)
    elif algo_name == "astar_e":
        events = run_astar(grid, start, goal, "euclidean", True)
    elif algo_name == "apf":
        events = run_apf(grid, start, goal)
    elif algo_name == "dstar":
        events = run_dstar(grid, start, goal)
    else:
        events = []

    return json.dumps(events)
`;
