import heapq
import time

# Peta grid dengan elevasi
terrain_map = [
    ['S', '1', '2', '3', '#', '4', '3', '2', '1', 'G'],
    ['1', '#', '2', '3', '#', '4', '#', '#', '2', '2'],
    ['1', '1', '1', '2', '3', '4', '5', '3', '#', '2'],
    ['#', '#', '2', '#', '2', '#', '3', '2', '1', '1'],
    ['1', '2', '3', '2', '1', '1', '2', '3', '4', '5']
]

ROWS = len(terrain_map)
COLS = len(terrain_map[0])
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]

# Cari posisi 'S' dan 'G'
def find_position(symbol):
    for r in range(ROWS):
        for c in range(COLS):
            if terrain_map[r][c] == symbol:
                return (r, c)
    return None

# Heuristik Manhattan
def manhattan(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

# Biaya berdasarkan elevasi
def elevation_cost(r, c):
    val = terrain_map[r][c]
    if val.isdigit():
        return int(val)
    return 1  # S atau G dianggap 1

# Fungsi pencarian (A* / GBFS)
def search(grid, start, goal, method='astar'):
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}
    explored_nodes = 0

    while frontier:
        _, current = heapq.heappop(frontier)
        explored_nodes += 1

        if current == goal:
            break

        for dr, dc in DIRECTIONS:
            nr, nc = current[0] + dr, current[1] + dc
            next_cell = (nr, nc)

            if 0 <= nr < ROWS and 0 <= nc < COLS and grid[nr][nc] != '#':
                new_cost = cost_so_far[current] + elevation_cost(nr, nc)

                if next_cell not in cost_so_far or new_cost < cost_so_far[next_cell]:
                    cost_so_far[next_cell] = new_cost
                    heuristic = manhattan(next_cell, goal)

                    if method == 'astar':
                        priority = new_cost + heuristic
                    elif method == 'gbfs':
                        priority = heuristic
                    else:
                        raise ValueError("Metode hanya 'astar' atau 'gbfs'.")

                    heapq.heappush(frontier, (priority, next_cell))
                    came_from[next_cell] = current

    return reconstruct_path(came_from, start, goal), explored_nodes

# Rekonstruksi jalur dari hasil pencarian
def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        if current not in came_from:
            return []
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path

# ========================
# Eksekusi dan Perbandingan
# ========================
if __name__ == "__main__":
    start = find_position('S')
    goal = find_position('G')
    results = {}

    for algo in ['astar', 'gbfs']:
        t0 = time.time()
        path, explored = search(terrain_map, start, goal, method=algo)
        t1 = time.time()
        elapsed = (t1 - t0) * 1000  # ms

        results[algo] = {
            "path": path,
            "time_ms": elapsed,
            "nodes": explored
        }

        print(f"\nMetode: {algo.upper()}")
        print("Path:", path)
        print(f"Time: {elapsed:.4f} ms")
        print(f"Jumlah node dieksplorasi: {explored}")

    # 📊 Ringkasan
    print("\n📊 Perbandingan Metode:")
    for method, data in results.items():
        print(f"{method.upper()} -> Waktu: {data['time_ms']:.4f} ms, Node: {data['nodes']}")
