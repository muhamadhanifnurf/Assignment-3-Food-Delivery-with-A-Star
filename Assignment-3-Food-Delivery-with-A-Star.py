import time

def manhattan(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

# Grid kota untuk Food Delivery
delivery_map = [
    ["S", ".", ".", ".", "T"],
    [".", "T", ".", "T", "."],
    [".", ".", ".", ".", "."],
    ["T", ".", "T", "T", "."],
    [".", ".", ".", ".", "G"]
]

# Temukan posisi start dan goal
start3, goal3 = None, None
for i in range(len(delivery_map)):
    for j in range(len(delivery_map[0])):
        if delivery_map[i][j] == "S":
            start3 = (i, j)
        elif delivery_map[i][j] == "G":
            goal3 = (i, j)

def a_star_grid(grid, start, goal):
    import time
    from heapq import heappush, heappop

    rows, cols = len(grid), len(grid[0])
    open_set = []
    heappush(open_set, (manhattan(start, goal), 0, start, [start]))
    visited = set()
    node_count = 0

    start_time = time.time()

    while open_set:
        f, g, current, path = heappop(open_set)
        node_count += 1
        if current == goal:
            elapsed_time = (time.time() - start_time) * 1000
            return path, g, node_count, elapsed_time
        if current in visited:
            continue
        visited.add(current)
        x, y = current
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < rows and 0 <= ny < cols:
                if grid[nx][ny] != "T" and (nx, ny) not in visited:
                    new_g = g + 1
                    new_f = new_g + manhattan((nx, ny), goal)
                    heappush(open_set, (new_f, new_g, (nx, ny), path + [(nx, ny)]))
    return None, float('inf'), node_count, 0

def gbfs_grid(grid, start, goal):
    import time
    from heapq import heappush, heappop

    rows, cols = len(grid), len(grid[0])
    open_set = []
    heappush(open_set, (manhattan(start, goal), start, [start]))
    visited = set()
    node_count = 0

    start_time = time.time()

    while open_set:
        h, current, path = heappop(open_set)
        node_count += 1
        if current == goal:
            elapsed_time = (time.time() - start_time) * 1000
            return path, node_count, elapsed_time
        if current in visited:
            continue
        visited.add(current)
        x, y = current
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < rows and 0 <= ny < cols:
                if grid[nx][ny] != "T" and (nx, ny) not in visited:
                    heappush(open_set, (manhattan((nx, ny), goal), (nx, ny), path + [(nx, ny)]))
    return None, node_count, 0

def visualize_grid(grid, path):
    """
    Visualisasi grid dengan jalur yang ditemukan.
    """
    visual_grid = [row[:] for row in grid]  # Salin grid
    for x, y in path:
        if visual_grid[x][y] not in ("S", "G"):
            visual_grid[x][y] = "*"
    print("\nGrid Visualization:")
    for row in visual_grid:
        print(" ".join(row))

# Menjalankan algoritma A* untuk menemukan jalur optimal pada grid kota
start_time = time.perf_counter()
path3_a_star, cost3_a_star, nodes3_a_star, elapsed3_a_star = a_star_grid(delivery_map, start3, goal3)
end_time = time.perf_counter()
elapsed3_a_star = (end_time - start_time) * 1000  # Hitung waktu eksekusi dalam milidetik

print("=== A* Search ===")
if path3_a_star:
    print("Path:", path3_a_star)
    print("Cost:", cost3_a_star)
    print("Nodes visited:", nodes3_a_star)
    print("Time (ms):", round(elapsed3_a_star, 4))  # Waktu dengan presisi tinggi
    visualize_grid(delivery_map, path3_a_star)
else:
    print("No path found using A*.")

# Menjalankan algoritma GBFS untuk menemukan jalur berdasarkan heuristik pada grid kota
start_time = time.perf_counter()
path3_gbfs, nodes3_gbfs, elapsed3_gbfs = gbfs_grid(delivery_map, start3, goal3)
end_time = time.perf_counter()
elapsed3_gbfs = (end_time - start_time) * 1000  # Hitung waktu eksekusi dalam milidetik

print("\n=== Greedy Best-First Search (GBFS) ===")
if path3_gbfs:
    print("Path:", path3_gbfs)
    print("Nodes visited:", nodes3_gbfs)
    print("Time (ms):", round(elapsed3_gbfs, 4))  # Waktu dengan presisi tinggi
    visualize_grid(delivery_map, path3_gbfs)
else:
    print("No path found using GBFS.")

# Membandingkan hasil algoritma A* dan GBFS berdasarkan panjang jalur, biaya, jumlah node yang dieksplorasi, dan waktu eksekusi
print("\n=== Metrics Report ===")
if path3_a_star and path3_gbfs:
    print("A* vs GBFS Comparison:")
    print(f"A* Path Length: {len(path3_a_star)}, Cost: {cost3_a_star}, Nodes Visited: {nodes3_a_star}, Time: {round(elapsed3_a_star, 3)} ms")
    print(f"GBFS Path Length: {len(path3_gbfs)}, Nodes Visited: {nodes3_gbfs}, Time: {round(elapsed3_gbfs, 3)} ms")
else:
    print("Comparison not possible. One or both algorithms failed to find a path.")
