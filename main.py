import heapq

class Node:
    def __init__(self, x, y, cost=0, heuristic=0):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.total_cost = cost + heuristic
        self.parent = None

    def __lt__(self, other):
        return self.total_cost < other.total_cost

def heuristic(current, goal):
    # Manhattan distance heuristic
    return abs(current.x - goal.x) + abs(current.y - goal.y)

def astar_search(grid, start, goal):
    rows = len(grid)
    cols = len(grid[0])
    
    # Directions for moving in 4-connected grid (up, down, left, right)
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    
    open_list = []
    heapq.heappush(open_list, start)
    
    visited = [[False] * cols for _ in range(rows)]
    visited[start.x][start.y] = True
    
    while open_list:
        current = heapq.heappop(open_list)
        
        if current.x == goal.x and current.y == goal.y:
            path = []
            while current.parent:
                path.append((current.x, current.y))
                current = current.parent
            path.append((start.x, start.y))
            path.reverse()
            return path
        
        for dir in directions:
            next_x = current.x + dir[0]
            next_y = current.y + dir[1]
            
            if 0 <= next_x < rows and 0 <= next_y < cols and not visited[next_x][next_y] and grid[next_x][next_y] == 0:
                new_cost = current.cost + 1
                new_heuristic = heuristic(Node(next_x, next_y), goal)
                next_node = Node(next_x, next_y, new_cost, new_heuristic)
                next_node.parent = current
                
                heapq.heappush(open_list, next_node)
                visited[next_x][next_y] = True
    
    return None  # No path found

# Example usage:
if __name__ == "__main__":
    grid = [
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0],
        [1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0]
    ]
    
    start = Node(0, 0)
    goal = Node(4, 4)
    
    path = astar_search(grid, start, goal)
    if path:
        print("Shortest path found:", path)
    else:
        print("No path found")
