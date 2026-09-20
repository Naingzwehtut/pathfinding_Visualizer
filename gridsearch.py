import heapq
import itertools
import random
import sys
import time
from collections import deque

import pygame


ROWS, COLS = 31, 31
CELL = 22
MARGIN = 20

GRID_W = COLS * CELL
GRID_H = ROWS * CELL
SIDEBAR_W = 380

WIN_W = MARGIN * 3 + GRID_W + SIDEBAR_W
WIN_H = MARGIN * 2 + GRID_H
FPS = 60

BG = (14, 16, 22)
PANEL_BG = (24, 27, 36)
PANEL_BORDER = (44, 50, 66)
CELL_OPEN = (30, 34, 44)
CELL_WALL = (58, 64, 80)
CELL_VISITED = (58, 106, 186)     
CELL_FRONTIER = (86, 190, 214)     
CELL_PATH = (255, 200, 45)        
CELL_START = (60, 200, 110)
CELL_GOAL = (232, 72, 72)

TEXT = (226, 232, 240)
TEXT_DIM = (126, 136, 155)
ACCENT = (110, 190, 255)
OK_GREEN = (86, 220, 140)
WARN = (255, 150, 90)

# node visual states
VIS_NONE, VIS_FRONTIER, VIS_VISITED, VIS_PATH = 0, 1, 2, 3

FONT_NAMES = "consolas,dejavusansmono,couriernew,monospace"



class Node:
    """A single grid cell."""

    __slots__ = ("row", "col", "wall", "vis", "g", "h", "f", "parent")

    def __init__(self, row: int, col: int) -> None:
        self.row = row
        self.col = col
        self.wall = False
        self.vis = VIS_NONE
        self.g = float("inf")
        self.h = 0.0
        self.f = float("inf")
        self.parent = None

    def soft_reset(self) -> None:
        """Clear search artefacts but keep wall flag."""
        self.vis = VIS_NONE
        self.g = float("inf")
        self.h = 0.0
        self.f = float("inf")
        self.parent = None


def get_neighbors(grid, node):
    """4-directional neighbours (up, right, down, left)."""
    r, c = node.row, node.col
    if r > 0:
        yield grid[r - 1][c]
    if c < COLS - 1:
        yield grid[r][c + 1]
    if r < ROWS - 1:
        yield grid[r + 1][c]
    if c > 0:
        yield grid[r][c - 1]


def manhattan(a: Node, b: Node) -> int:
    return abs(a.row - b.row) + abs(a.col - b.col)


def reconstruct_path(node: Node):
    """Walk parent pointers back to the start (exclusive)."""
    path = []
    while node.parent is not None:
        path.append(node)
        node = node.parent
    path.reverse()
    return path


def alg_bfs(grid, start: Node, goal: Node):
    """Breadth-First Search – FIFO queue, optimal on unweighted graphs."""
    start.parent = None
    frontier = deque([start])
    in_frontier = {start}
    closed = set()

    yield ("frontier", start)

    while frontier:
        current = frontier.popleft()
        in_frontier.discard(current)
        closed.add(current)
        yield ("expand", current)

        if current is goal:
            yield ("path", reconstruct_path(current))
            return

        for nb in get_neighbors(grid, current):
            if nb.wall or nb in closed or nb in in_frontier:
                continue
            nb.parent = current
            in_frontier.add(nb)
            frontier.append(nb)
            yield ("frontier", nb)

    yield ("path", [])


def alg_dfs(grid, start: Node, goal: Node):
    """Depth-First Search – LIFO stack, not optimal."""
    start.parent = None
    stack = [start]
    in_stack = {start}
    closed = set()

    yield ("frontier", start)

    while stack:
        current = stack.pop()
        if current in closed:
            continue
        closed.add(current)
        in_stack.discard(current)
        yield ("expand", current)

        if current is goal:
            yield ("path", reconstruct_path(current))
            return

        for nb in get_neighbors(grid, current):
            if nb.wall or nb in closed or nb in in_stack:
                continue
            nb.parent = current
            in_stack.add(nb)
            stack.append(nb)
            yield ("frontier", nb)

    yield ("path", [])


def alg_dijkstra(grid, start: Node, goal: Node):
    """Dijkstra – min-heap ordered by g, optimal for uniform costs."""
    start.parent = None
    start.g = 0
    counter = itertools.count()
    pq = [(0, next(counter), start)]
    closed = set()
    in_frontier = {start}

    yield ("frontier", start)

    while pq:
        _, _, current = heapq.heappop(pq)
        if current in closed:
            continue
        closed.add(current)
        in_frontier.discard(current)
        yield ("expand", current)

        if current is goal:
            yield ("path", reconstruct_path(current))
            return

        for nb in get_neighbors(grid, current):
            if nb.wall or nb in closed:
                continue
            ng = current.g + 1
            if ng < nb.g:
                nb.g = ng
                nb.parent = current
                heapq.heappush(pq, (ng, next(counter), nb))
                if nb not in in_frontier:
                    in_frontier.add(nb)
                yield ("frontier", nb)

    yield ("path", [])


def alg_astar(grid, start: Node, goal: Node):
    """A* – min-heap ordered by f = g + h (Manhattan), optimal & complete."""
    start.parent = None
    start.g = 0
    start.h = manhattan(start, goal)
    start.f = start.g + start.h

    counter = itertools.count()
    pq = [(start.f, next(counter), start)]
    closed = set()
    in_frontier = {start}

    yield ("frontier", start)

    while pq:
        _, _, current = heapq.heappop(pq)
        if current in closed:
            continue
        closed.add(current)
        in_frontier.discard(current)
        yield ("expand", current)

        if current is goal:
            yield ("path", reconstruct_path(current))
            return

        for nb in get_neighbors(grid, current):
            if nb.wall or nb in closed:
                continue
            ng = current.g + 1
            if ng < nb.g:
                nb.g = ng
                nb.h = manhattan(nb, goal)
                nb.f = nb.g + nb.h
                nb.parent = current
                heapq.heappush(pq, (nb.f, next(counter), nb))
                if nb not in in_frontier:
                    in_frontier.add(nb)
                yield ("frontier", nb)

    yield ("path", [])


def alg_greedy(grid, start: Node, goal: Node):
    """Greedy Best-First – min-heap ordered purely by heuristic h."""
    start.parent = None
    start.h = manhattan(start, goal)

    counter = itertools.count()
    pq = [(start.h, next(counter), start)]
    closed = set()
    in_frontier = {start}

    yield ("frontier", start)

    while pq:
        _, _, current = heapq.heappop(pq)
        if current in closed:
            continue
        closed.add(current)
        in_frontier.discard(current)
        yield ("expand", current)

        if current is goal:
            yield ("path", reconstruct_path(current))
            return

        for nb in get_neighbors(grid, current):
            if nb.wall or nb in closed or nb in in_frontier:
                continue
            nb.h = manhattan(nb, goal)
            nb.parent = current
            in_frontier.add(nb)
            heapq.heappush(pq, (nb.h, next(counter), nb))
            yield ("frontier", nb)

    yield ("path", [])


ALGO_ORDER = ["BFS", "DFS", "Dijkstra", "A*", "Greedy Best-First"]

ALGORITHMS = {
    "BFS": alg_bfs,
    "DFS": alg_dfs,
    "Dijkstra": alg_dijkstra,
    "A*": alg_astar,
    "Greedy Best-First": alg_greedy,
}

ALGO_INFO = {
    "BFS": {
        "full": "Breadth-First Search",
        "time": "O(V + E)",
        "space": "O(V)",
        "frontier": "FIFO Queue",
        "optimal": "Yes (unweighted)",
    },
    "DFS": {
        "full": "Depth-First Search",
        "time": "O(V + E)",
        "space": "O(V)",
        "frontier": "LIFO Stack",
        "optimal": "No",
    },
    "Dijkstra": {
        "full": "Dijkstra's Algorithm",
        "time": "O(E log V)",
        "space": "O(V)",
        "frontier": "Min-Heap (g)",
        "optimal": "Yes",
    },
    "A*": {
        "full": "A* Search",
        "time": "O(E log V)",
        "space": "O(V)",
        "frontier": "Min-Heap (g+h)",
        "optimal": "Yes (admissible h)",
    },
    "Greedy Best-First": {
        "full": "Greedy Best-First",
        "time": "O(E log V)",
        "space": "O(V)",
        "frontier": "Min-Heap (h)",
        "optimal": "No",
    },
}



class Visualizer:
    """Owns the grid, the search session and all rendering / input."""

    def __init__(self) -> None:
        pygame.init()
        pygame.display.set_caption("Pathfinding Visualizer")
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        self.clock = pygame.time.Clock()

      
        self.f_title = pygame.font.SysFont(FONT_NAMES, 19, bold=True)
        self.f_head = pygame.font.SysFont(FONT_NAMES, 13, bold=True)
        self.f_value = pygame.font.SysFont(FONT_NAMES, 15, bold=True)
        self.f_label = pygame.font.SysFont(FONT_NAMES, 14)
        self.f_small = pygame.font.SysFont(FONT_NAMES, 13)
        self.f_cell = pygame.font.SysFont(FONT_NAMES, 13, bold=True)

       
        self.grid = [[Node(r, c) for c in range(COLS)] for r in range(ROWS)]
        self.start = (1, 1)
        self.goal = (ROWS - 2, COLS - 2)

    
        self.algorithm = "A*"
        self.gen = None
        self.running = False
        self.status = "Idle"
        self.expanded = 0
        self.frontier_max = 0
        self.frontier_set = set()
        self.path_len = 0
        self.elapsed_ms = 0.0
        self.speed = 3            

        self.drag = None         


    def reset_search(self) -> None:
        """Clear every visual / algorithmic artefact but keep walls."""
        self.gen = None
        self.running = False
        self.status = "Idle"
        self.expanded = 0
        self.frontier_max = 0
        self.frontier_set = set()
        self.path_len = 0
        self.elapsed_ms = 0.0
        for row in self.grid:
            for node in row:
                node.soft_reset()

    def clear_board(self) -> None:
        """Remove walls and reset the search."""
        for row in self.grid:
            for node in row:
                node.wall = False
        self.reset_search()

    def generate_maze(self) -> None:
        """Carve a perfect maze with randomised recursive backtracking."""
        for row in self.grid:
            for node in row:
                node.wall = True

        stack = [(1, 1)]
        self.grid[1][1].wall = False
        visited = {(1, 1)}

        while stack:
            r, c = stack[-1]
            options = []
            for dr, dc in ((-2, 0), (2, 0), (0, -2), (0, 2)):
                nr, nc = r + dr, c + dc
                if 0 < nr < ROWS - 1 and 0 < nc < COLS - 1 and (nr, nc) not in visited:
                    options.append((nr, nc, dr, dc))
            if options:
                nr, nc, dr, dc = random.choice(options)
                self.grid[r + dr // 2][c + dc // 2].wall = False
                self.grid[nr][nc].wall = False
                visited.add((nr, nc))
                stack.append((nr, nc))
            else:
                stack.pop()

        self.start = (1, 1)
        self.goal = (ROWS - 2 if (ROWS - 2) % 2 else ROWS - 3,
                     COLS - 2 if (COLS - 2) % 2 else COLS - 3)
        self.reset_search()

    def set_algorithm(self, name: str) -> None:
        if name != self.algorithm:
            self.algorithm = name
            self.reset_search()

    def toggle_search(self) -> None:
        """SPACE behaviour: start / pause / resume."""
        if self.status == "Searching":
            self.running = False
            self.status = "Paused"
            return
        if self.status == "Paused":
            self.running = True
            self.status = "Searching"
            return


        self.reset_search()
        s = self.grid[self.start[0]][self.start[1]]
        g = self.grid[self.goal[0]][self.goal[1]]
        self.gen = ALGORITHMS[self.algorithm](self.grid, s, g)
        self.running = True
        self.status = "Searching"

  
    def update(self) -> None:
        if not self.running or self.gen is None:
            return

        t0 = time.perf_counter()
        for _ in range(self.speed):
            try:
                event, payload = next(self.gen)
            except StopIteration:
                self.running = False
                if self.status == "Searching":
                    self.status = "Finished"
                break

            self._apply(event, payload)
            if not self.running:
                break
        self.elapsed_ms += (time.perf_counter() - t0) * 1000.0

    def _apply(self, event: str, payload) -> None:
        if event == "frontier":
            node = payload
            if node not in self.frontier_set:
                self.frontier_set.add(node)
                if node.vis != VIS_VISITED:
                    node.vis = VIS_FRONTIER
                self.frontier_max = max(self.frontier_max, len(self.frontier_set))

        elif event == "expand":
            node = payload
            self.frontier_set.discard(node)
            node.vis = VIS_VISITED
            self.expanded += 1

        elif event == "path":
            path = payload
            for node in path:
                node.vis = VIS_PATH
            self.path_len = len(path)
            self.running = False
            self.status = "Finished" if (path or self.start == self.goal) else "No Path"


    def handle_events(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._quit()
            elif event.type == pygame.KEYDOWN:
                self._on_key(event.key)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                self._on_mouse_down(event)
            elif event.type == pygame.MOUSEMOTION:
                self._on_mouse_motion(event)
            elif event.type == pygame.MOUSEBUTTONUP:
                self.drag = None

    def _on_key(self, key: int) -> None:
        if key == pygame.K_ESCAPE:
            self._quit()
        elif key == pygame.K_1:
            self.set_algorithm("BFS")
        elif key == pygame.K_2:
            self.set_algorithm("DFS")
        elif key == pygame.K_3:
            self.set_algorithm("Dijkstra")
        elif key == pygame.K_4:
            self.set_algorithm("A*")
        elif key == pygame.K_5:
            self.set_algorithm("Greedy Best-First")
        elif key == pygame.K_SPACE:
            self.toggle_search()
        elif key == pygame.K_r:
            self.reset_search()
        elif key == pygame.K_c:
            self.clear_board()
        elif key == pygame.K_m:
            self.generate_maze()
        elif key == pygame.K_UP:
            self.speed = min(60, self.speed + 1)
        elif key == pygame.K_DOWN:
            self.speed = max(1, self.speed - 1)

    def _on_mouse_down(self, event) -> None:
        cell = self.cell_at(event.pos)
        if cell is None:
            return

        if event.button == 1:
            if cell == self.start:
                self.drag = "start"
            elif cell == self.goal:
                self.drag = "goal"
            else:
                self.drag = "wall"
                self._set_wall(cell, True)
        elif event.button == 3:
            self.drag = "erase"
            self._set_wall(cell, False)

    def _on_mouse_motion(self, event) -> None:
        if self.drag is None:
            return
        cell = self.cell_at(event.pos)
        if cell is None:
            return

        if self.drag == "wall":
            self._set_wall(cell, True)
        elif self.drag == "erase":
            self._set_wall(cell, False)
        elif self.drag == "start":
            if cell != self.goal and cell != self.start and not self.grid[cell[0]][cell[1]].wall:
                self.start = cell
                self.reset_search()
        elif self.drag == "goal":
            if cell != self.start and cell != self.goal and not self.grid[cell[0]][cell[1]].wall:
                self.goal = cell
                self.reset_search()

    def _set_wall(self, cell, value: bool) -> None:
        if cell == self.start or cell == self.goal:
            return
        node = self.grid[cell[0]][cell[1]]
        if node.wall != value:
            node.wall = value
            if self.status != "Idle":
                self.reset_search()

    def cell_at(self, pos):
        x, y = pos
        if x < MARGIN or y < MARGIN:
            return None
        c = (x - MARGIN) // CELL
        r = (y - MARGIN) // CELL
        if 0 <= r < ROWS and 0 <= c < COLS:
            return (r, c)
        return None


    def draw(self) -> None:
        self.screen.fill(BG)
        self.draw_grid()
        self.draw_sidebar()
        pygame.display.flip()

    def _cell_color(self, node: Node, r: int, c: int):
        if (r, c) == self.start:
            return CELL_START
        if (r, c) == self.goal:
            return CELL_GOAL
        if node.wall:
            return CELL_WALL
        if node.vis == VIS_PATH:
            return CELL_PATH
        if node.vis == VIS_VISITED:
            return CELL_VISITED
        if node.vis == VIS_FRONTIER:
            return CELL_FRONTIER
        return CELL_OPEN

    def draw_grid(self) -> None:
        board = pygame.Rect(MARGIN - 6, MARGIN - 6, GRID_W + 12, GRID_H + 12)
        pygame.draw.rect(self.screen, PANEL_BG, board, border_radius=10)
        pygame.draw.rect(self.screen, PANEL_BORDER, board, 2, border_radius=10)

        blit = self.screen.blit
        draw_rect = pygame.draw.rect

        for r in range(ROWS):
            y = MARGIN + r * CELL
            for c in range(COLS):
                node = self.grid[r][c]
                rect = pygame.Rect(MARGIN + c * CELL, y, CELL, CELL)
                draw_rect(self.screen, self._cell_color(node, r, c),
                          rect.inflate(-2, -2), border_radius=4)

        for (r, c), label in ((self.start, "S"), (self.goal, "G")):
            surf = self.f_cell.render(label, True, (255, 255, 255))
            cx = MARGIN + c * CELL + CELL // 2
            cy = MARGIN + r * CELL + CELL // 2
            blit(surf, (cx - surf.get_width() // 2, cy - surf.get_height() // 2))

    def _text(self, text, x, y, font, color):
        self.screen.blit(font.render(text, True, color), (x, y))

    def _divider(self, x, y, w):
        pygame.draw.line(self.screen, PANEL_BORDER, (x, y), (x + w, y), 1)
        return y + 14

    def _stat(self, label, value, x, right, y, color=None):
        self._text(label, x, y, self.f_label, TEXT_DIM)
        surf = self.f_value.render(value, True, color or TEXT)
        self.screen.blit(surf, (right - surf.get_width(), y - 2))
        return y + 23

    def _control(self, key, desc, x, y):
        self._text(key, x, y, self.f_label, ACCENT)
        self._text(desc, x + 92, y, self.f_label, TEXT_DIM)
        return y + 20

    def draw_sidebar(self) -> None:
        x0 = MARGIN * 2 + GRID_W
        panel = pygame.Rect(x0, MARGIN, SIDEBAR_W, GRID_H)
        pygame.draw.rect(self.screen, PANEL_BG, panel, border_radius=12)
        pygame.draw.rect(self.screen, PANEL_BORDER, panel, 2, border_radius=12)

        x = x0 + 22
        right = x0 + SIDEBAR_W - 22
        y = MARGIN + 16

        self._text("PATHFINDING VISUALIZER", x, y, self.f_title, TEXT)
        y += 28
        self._text("BFS · DFS · Dijkstra · A* · Greedy", x, y, self.f_small, TEXT_DIM)
        y += 24
        y = self._divider(x, y, SIDEBAR_W - 44)

        self._text("ALGORITHM", x, y, self.f_head, ACCENT)
        y += 20
        idx = ALGO_ORDER.index(self.algorithm) + 1
        self._text(f"{idx}  ·  {ALGO_INFO[self.algorithm]['full']}", x, y, self.f_value, TEXT)
        y += 28
        y = self._divider(x, y, SIDEBAR_W - 44)


        self._text("LIVE STATS", x, y, self.f_head, ACCENT)
        y += 22

        status_color = {
            "Idle": TEXT_DIM,
            "Searching": CELL_FRONTIER,
            "Paused": WARN,
            "Finished": OK_GREEN,
            "No Path": CELL_GOAL,
        }.get(self.status, TEXT)

        open_cells = sum(1 for row in self.grid for n in row if not n.wall)

        y = self._stat("Status", self.status, x, right, y, status_color)
        y = self._stat("Nodes Expanded", str(self.expanded), x, right, y)
        y = self._stat("Frontier Max", str(self.frontier_max), x, right, y)
        y = self._stat("Path Length", str(self.path_len), x, right, y)
        y = self._stat("Execution Time", f"{self.elapsed_ms:.1f} ms", x, right, y)
        y = self._stat("Open Cells (V)", str(open_cells), x, right, y)
        y = self._divider(x, y, SIDEBAR_W - 44)

       
        info = ALGO_INFO[self.algorithm]
        self._text("ALGORITHM INFO", x, y, self.f_head, ACCENT)
        y += 22
        y = self._stat("Time Complexity", info["time"], x, right, y)
        y = self._stat("Space Complexity", info["space"], x, right, y)
        y = self._stat("Frontier Type", info["frontier"], x, right, y)
        y = self._stat("Optimal", info["optimal"], x, right, y)
        y = self._divider(x, y, SIDEBAR_W - 44)

     
        self._text("CONTROLS", x, y, self.f_head, ACCENT)
        y += 22
        y = self._control("1 - 5", "Select algorithm", x, y)
        y = self._control("SPACE", "Start / Pause search", x, y)
        y = self._control("R", "Reset search state", x, y)
        y = self._control("C", "Clear board", x, y)
        y = self._control("M", "Random maze", x, y)
        y = self._control("LMB", "Draw wall / drag S,G", x, y)
        y = self._control("RMB", "Erase wall", x, y)
        self._control("UP / DN", f"Speed: {self.speed} step/frame", x, y)


    def run(self) -> None:
        while True:
            self.handle_events()
            self.update()
            self.draw()
            self.clock.tick(FPS)

    @staticmethod
    def _quit() -> None:
        pygame.quit()
        sys.exit(0)


def main() -> None:
    Visualizer().run()


if __name__ == "__main__":
    main()