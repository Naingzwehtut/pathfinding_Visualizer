# Pathfinding Visualizer

An interactive Python application for visualizing and comparing classic pathfinding algorithms on a 2D grid. Built with **Pygame**.

![Python](https://img.shields.io/badge/Python-3.8%2B-blue)
![Pygame](https://img.shields.io/badge/Pygame-2.x-green)

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Running](#running)
- [Controls](#controls)
- [Visual Legend](#visual-legend)
- [Algorithms](#algorithms)
- [Project Structure](#project-structure)
- [How It Works](#how-it-works)
- [Customization](#customization)
- [License](#license)

---

## Overview

Pathfinding Visualizer lets you draw walls, move start/goal nodes, and watch five different search algorithms explore the grid in real time. The sidebar displays live statistics such as nodes expanded, frontier size, path length, and execution time, alongside complexity information for the selected algorithm.

The provided code should be saved as a Python file, for example:

```bash
pathfinding_visualizer.py
```

> If your file is currently named `import heapq.txt`, rename it to `pathfinding_visualizer.py` before running.

---

## Features

- **5 Algorithms** — BFS, DFS, Dijkstra, A*, and Greedy Best-First Search
- **Real-time animation** — watch the frontier expand and the path unfold
- **Interactive editing** — draw/erase walls, drag the start and goal nodes
- **Random maze generation** — randomized recursive backtracking carver
- **Live statistics** — expanded nodes, max frontier size, path length, execution time
- **Adjustable speed** — from 1 step per frame up to 60
- **Algorithm info panel** — time/space complexity, frontier type, optimality

---

## Requirements

- Python 3.8 or newer
- [pygame](https://www.pygame.org/) 2.x

---

## Installation

Install dependencies:

```bash
pip install pygame
```

---

## Running

Save the script as `pathfinding_visualizer.py` and run:

```bash
python pathfinding_visualizer.py
```

---

## Controls

| Input | Action |
|-------|--------|
| `1` – `5` | Select algorithm: BFS, DFS, Dijkstra, A*, Greedy |
| `SPACE` | Start / Pause / Resume search |
| `R` | Reset search state, keeps walls |
| `C` | Clear board, removes walls |
| `M` | Generate a random maze |
| `↑` / `↓` | Increase / decrease animation speed |
| **Left Mouse** | Draw walls · drag **S** start or **G** goal |
| **Right Mouse** | Erase walls |
| `ESC` | Quit |

---

## Visual Legend

| Color | Meaning |
|-------|---------|
| 🟩 Green | Start node `S` |
| 🟥 Red | Goal node `G` |
| ⬛ Dark grey | Wall |
| 🔵 Blue | Visited / expanded node |
| 🩵 Cyan | Frontier node |
| 🟨 Yellow | Final path |
| ⬛ Dark | Empty cell |

---

## Algorithms

| Algorithm | Frontier | Time | Space | Optimal |
|-----------|----------|------|-------|---------|
| **BFS** | FIFO Queue | `O(V + E)` | `O(V)` | Yes, unweighted |
| **DFS** | LIFO Stack | `O(V + E)` | `O(V)` | No |
| **Dijkstra** | Min-Heap `g` | `O(E log V)` | `O(V)` | Yes |
| **A\*** | Min-Heap `g + h` | `O(E log V)` | `O(V)` | Yes, admissible `h` |
| **Greedy Best-First** | Min-Heap `h` | `O(E log V)` | `O(V)` | No |

A* and Greedy use the **Manhattan distance** heuristic, which is admissible for 4-directional movement on a uniform-cost grid.

---

## Project Structure

```text
pathfinding_visualizer.py
│
├── Node                 # Grid cell: row, col, wall, search state
├── get_neighbors()      # 4-directional neighbour generator
├── manhattan()          # Heuristic function
├── reconstruct_path()   # Walks parent pointers back to start
│
├── alg_bfs()            # Breadth-First Search generator
├── alg_dfs()            # Depth-First Search generator
├── alg_dijkstra()       # Dijkstra's algorithm generator
├── alg_astar()          # A* Search generator
├── alg_greedy()         # Greedy Best-First generator
│
├── ALGO_ORDER / ALGORITHMS / ALGO_INFO
│
└── Visualizer           # Main class: grid, session, rendering, input
    ├── reset_search() / clear_board() / generate_maze()
    ├── toggle_search()  # SPACE behaviour
    ├── update()         # steps the generator N times per frame
    ├── _apply()         # translates algorithm events into visuals
    ├── handle_events() / _on_key() / _on_mouse_*()
    ├── draw() / draw_grid() / draw_sidebar()
    └── run()            # main loop
```

---

## How It Works

Each algorithm is implemented as a **generator** that yields event tuples such as:

```python
("frontier", node)
("expand", node)
("path", [nodes])
```

The `Visualizer.update()` method drains this generator at a rate controlled by `self.speed`, which cleanly decouples the algorithm logic from the animation loop.

This design makes it easy to add new algorithms without changing the rendering system.

---

## Customization

### Add a New Algorithm

1. Write a generator:

   ```python
   def alg_youralgo(grid, start, goal):
       ...
       yield ("frontier", node)
       yield ("expand", node)
       yield ("path", path)
   ```

2. Register it in `ALGORITHMS` and `ALGO_ORDER`.

3. Add an entry to `ALGO_INFO` with complexity details.

4. Bind a key in `_on_key()`.

### Change Grid Size

Modify the constants near the top of the file:

```python
ROWS, COLS = 31, 31
CELL = 22
```

Maze generation works best with odd dimensions.

---

## License

This project is provided as-is for educational purposes. Use and modify freely.
