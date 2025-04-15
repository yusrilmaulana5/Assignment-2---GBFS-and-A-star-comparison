import heapq
import time
import random

directions = [(-1,0), (1,0), (0,-1), (0,1)]

def generate_grid(size, num_obstacles):
    side = int(size ** 0.5)
    grid = [[0 for _ in range(side)] for _ in range(side)]
    count = 0
    while count < num_obstacles:
        x, y = random.randint(0, side-1), random.randint(0, side-1)
        if grid[x][y] == 0:
            grid[x][y] = 1
            count += 1
    return grid, side

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def gbfs_time(grid, start, goal):
    visited = set()
    pq = []
    heapq.heappush(pq, (heuristic(start, goal), start))
    while pq:
        _, current = heapq.heappop(pq)
        if current == goal:
            break
        if current in visited:
            continue
        visited.add(current)
        for dx, dy in directions:
            nx, ny = current[0] + dx, current[1] + dy
            if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]):
                if grid[nx][ny] == 0 and (nx, ny) not in visited:
                    heapq.heappush(pq, (heuristic((nx, ny), goal), (nx, ny)))

def a_star_time(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    g_score = {start: 0}
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            break
        for dx, dy in directions:
            nx, ny = current[0] + dx, current[1] + dy
            neighbor = (nx, ny)
            if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]):
                if grid[nx][ny] == 1:
                    continue
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

# Eksperimen
experiments = [
    (5000, 10),
    (50000, 100),
    (500000, 1000),
    (5000000, 10000),
    (50000000, 100000)
]

print("Time Comparison (in milliseconds)")
print(f"{'Experiment':45} {'GBFS':>10} {'A*':>10}")
for i, (num_nodes, num_obstacles) in enumerate(experiments):
    grid, side = generate_grid(num_nodes, num_obstacles)
    start, goal = (0, 0), (side-1, side-1)
    
    t0 = time.time()
    gbfs_time(grid, start, goal)
    gbfs_elapsed = (time.time() - t0) * 1000
    
    t1 = time.time()
    a_star_time(grid, start, goal)
    a_star_elapsed = (time.time() - t1) * 1000
    
    print(f"# {i+1} Nodes: {num_nodes:<8} Obstacles: {num_obstacles:<6}  {gbfs_elapsed:10.2f} {a_star_elapsed:10.2f}")