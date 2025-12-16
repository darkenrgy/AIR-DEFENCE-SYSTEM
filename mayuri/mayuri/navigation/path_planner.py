import heapq
import math
import json
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String



class AStarPlanner:
    """Simple 2D A* grid planner."""

    def __init__(self, grid, resolution=1.0):
        self.grid = grid  
        self.resolution = resolution
        self.rows = len(grid)
        self.cols = len(grid[0]) if self.rows else 0

    def heuristic(self, a, b):
        """Euclidean distance."""
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def neighbors(self, node):
        """Return 8-connected neighbors."""
        (x, y) = node
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                       (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.rows and 0 <= ny < self.cols and self.grid[nx][ny] == 0:
                yield (nx, ny)

    def plan(self, start, goal):
        """Run A* to find a path from start to goal."""
        if not (0 <= start[0] < self.rows and 0 <= start[1] < self.cols):
            raise ValueError("Start outside grid bounds")
        if not (0 <= goal[0] < self.rows and 0 <= goal[1] < self.cols):
            raise ValueError("Goal outside grid bounds")

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.neighbors(current):
                tentative_g = g_score[current] + self.heuristic(current, neighbor)
                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return []

    def reconstruct_path(self, came_from, current):
        """Backtrack to reconstruct full path."""
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        total_path.reverse()
        return total_path



class PathPlanner(Node):
    def __init__(self):
        super().__init__("path_planner")

        # Parameters
        self.declare_parameter("grid_size", [50, 50])
        self.declare_parameter("cell_resolution", 1.0)
        self.declare_parameter("map_update_rate", 1.0)

        size = self.get_parameter("grid_size").value
        self.grid_size = (int(size[0]), int(size[1]))
        self.resolution = float(self.get_parameter("cell_resolution").value)
        self.map_update_rate = float(self.get_parameter("map_update_rate").value)

        # Initialize grid (0 = free, 1 = obstacle)
        self.grid = [[0 for _ in range(self.grid_size[1])] for _ in range(self.grid_size[0])]
        self.lock = threading.Lock()

        # Publishers / Subscribers
        self.path_pub = self.create_publisher(String, "/mayuri/path_planner/path", 10)
        self.create_subscription(String, "/mayuri/navigation/request", self.on_path_request, 10)
        self.create_subscription(String, "/mayuri/sensor/obstacle_map", self.on_obstacle_update, 10)

        # Background thread for map health checks
        self.running = True
        threading.Thread(target=self._map_health_loop, daemon=True).start()

        self.get_logger().info(" PathPlanner initialized and ready.")

    # ---------------------------------------------------------
    def on_obstacle_update(self, msg: String):
        """Update obstacle grid from sensor fusion data."""
        try:
            data = json.loads(msg.data)
            obstacles = data.get("obstacles", [])
            with self.lock:
                # Reset grid
                for x in range(self.grid_size[0]):
                    for y in range(self.grid_size[1]):
                        self.grid[x][y] = 0
                # Mark obstacles
                for ox, oy in obstacles:
                    if 0 <= ox < self.grid_size[0] and 0 <= oy < self.grid_size[1]:
                        self.grid[ox][oy] = 1
        except Exception as e:
            self.get_logger().warning(f"Obstacle update error: {e}")

    # ---------------------------------------------------------
    def on_path_request(self, msg: String):
        """Handle incoming path planning requests."""
        try:
            req = json.loads(msg.data)
            start = tuple(req.get("start", (0, 0)))
            goal = tuple(req.get("goal", (10, 10)))
            self.get_logger().info(f"Path request: {start} → {goal}")

            with self.lock:
                planner = AStarPlanner(self.grid, resolution=self.resolution)
                path = planner.plan(start, goal)

            if not path:
                self.get_logger().warning(" No valid path found.")
                self.publish_path([], success=False)
            else:
                self.publish_path(path, success=True)

        except Exception as e:
            self.get_logger().error(f"Path planning error: {e}")

    # ---------------------------------------------------------
    def publish_path(self, path, success=True):
        """Publish computed path."""
        msg = {
            "timestamp": time.time(),
            "success": success,
            "path": [{"x": p[0], "y": p[1]} for p in path]
        }
        m = String()
        m.data = json.dumps(msg)
        self.path_pub.publish(m)
        if success:
            self.get_logger().info(f"Path published ({len(path)} points).")

    # ---------------------------------------------------------
    def _map_health_loop(self):
        """Diagnostics thread to monitor obstacle grid activity."""
        while self.running and rclpy.ok():
            time.sleep(1.0 / max(0.01, self.map_update_rate))
            active_cells = sum(sum(row) for row in self.grid)
            if active_cells > 0:
                self.get_logger().debug(f"Active obstacles: {active_cells}")

    # ---------------------------------------------------------
    def destroy_node(self):
        self.running = False
        self.get_logger().info(" PathPlanner shutting down.")
        super().destroy_node()


# -------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PathPlanner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
