# SELF_STUDY — Tài liệu tự học chi tiết

> **Hướng dẫn:** Đọc tuần tự từ Part 1 đến Part 8. Mỗi part có lý thuyết, sơ đồ, công thức, và bài tập.
> Ước tính thời gian: 2–3 giờ đọc mỗi part, tổng cộng ~20 giờ cho toàn bộ tài liệu.

---

# MỤC LỤC

- [Part 1: Tổng quan hệ thống](#part-1-tổng-quan-hệ-thống)
- [Part 2: Grid Map và Biểu diễn môi trường](#part-2-grid-map-và-biểu-diễn-môi-trường)
- [Part 3: Thuật toán A* — Tìm đường ngắn nhất](#part-3-thuật-toán-a--tìm-đường-ngắn-nhất)
- [Part 4: Trajectory Generation và Smoothing](#part-4-trajectory-generation-và-smoothing)
- [Part 5: Mô hình động học xe — Kinematic Bicycle Model](#part-5-mô-hình-động-học-xe--kinematic-bicycle-model)
- [Part 6: Model Predictive Control (MPC)](#part-6-model-predictive-control-mpc)
- [Part 7: Simulation và Evaluation](#part-7-simulation-và-evaluation)
- [Part 8: AI Agent và MCP](#part-8-ai-agent-và-mcp)

---

# Part 1: Tổng quan hệ thống

## 1.1 Bài toán là gì?

Cho một robot di động trên mặt phẳng 2D, tìm đường đi từ điểm A đến điểm B sao cho:
- Không va chạm với vật cản
- Bám theo quỹ đạo tham chiếu một cách chính xác
- Tín hiệu điều khiển mượt (không giật)

## 1.2 Kiến trúc tổng quan

```
┌─────────────────────────────────────────────────────────────────────┐
│                        MPC AGENTIC ROBOTICS                         │
│                                                                     │
│  ┌──────────┐    ┌──────────────┐    ┌───────────────────────────┐  │
│  │          │    │              │    │                           │  │
│  │  MAP     │───▶│   PLANNER    │───▶│      CONTROLLER           │  │
│  │  (Grid)  │    │   (A*)       │    │  (Iterative MPC + CVXPY)  │  │
│  │          │    │              │    │                           │  │
│  └──────────┘    └──────────────┘    └───────────┬───────────────┘  │
│                                                  │                  │
│                                                  ▼                  │
│  ┌──────────┐    ┌──────────────┐    ┌───────────────────────────┐  │
│  │          │    │              │    │                           │  │
│  │  OUTPUT  │◀───│  EVALUATION  │◀───│      SIMULATOR            │  │
│  │  (Log,   │    │  (Metrics)   │    │  (2D Grid + Collision)    │  │
│  │   Plot)  │    │              │    │                           │  │
│  └──────────┘    └──────────────┘    └───────────┬───────────────┘  │
│                                                  ▲                  │
│                                                  │                  │
│  ┌──────────┐    ┌──────────────┐                │                  │
│  │          │    │              │                │                  │
│  │   AI     │◀──▶│  MCP SERVER  │────────────────┘                  │
│  │  AGENT   │    │  (Tools)     │                                   │
│  │          │    │              │                                   │
│  └──────────┘    └──────────────┘                                   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## 1.3 Luồng dữ liệu chi tiết

```
                        ┌─────────────────────┐
                        │    MAP (YAML)        │
                        │  name, width, height │
                        │  resolution,         │
                        │  obstacles[]         │
                        └──────────┬──────────┘
                                   │
                                   ▼
                        ┌─────────────────────┐
                        │   LOAD MAP          │
                        │  → grid ndarray     │
                        │  (0=free, 1=wall)   │
                        └──────────┬──────────┘
                                   │
                    ┌──────────────┼──────────────┐
                    ▼              ▼              ▼
              ┌──────────┐  ┌──────────┐  ┌──────────┐
              │  START   │  │   A*     │  │   GOAL   │
              │  (r,c)   │  │ ALGORITHM│  │  (r,c)   │
              └──────────┘  └────┬─────┘  └──────────┘
                                 │
                                 ▼
                        ┌─────────────────────┐
                        │  PATH (cell list)   │
                        │  [(0,0),(1,1),...]  │
                        └──────────┬──────────┘
                                   │
                                   ▼
                        ┌─────────────────────┐
                        │  PATH_TO_CELLS()    │
                        │  → world coords (x,y)│
                        └──────────┬──────────┘
                                   │
                                   ▼
                        ┌─────────────────────┐
                        │  TRAJECTORY         │
                        │  t, x, y, yaw, v    │
                        │  + smoothing        │
                        └──────────┬──────────┘
                                   │
                                   ▼
                        ┌─────────────────────┐
                        │  MPC CONTROLLER     │
                        │                     │
                        │  for each step:     │
                        │    ref_window → MPC │
                        │    MPC → control    │
                        │    control → step   │
                        │    step → new state │
                        └──────────┬──────────┘
                                   │
                                   ▼
                        ┌─────────────────────┐
                        │  SIMULATION LOG     │
                        │  t,x,y,yaw,v,      │
                        │  steer,accel,       │
                        │  collision,         │
                        │  min_dist           │
                        └──────────┬──────────┘
                                   │
                    ┌──────────────┼──────────────┐
                    ▼              ▼              ▼
              ┌──────────┐  ┌──────────┐  ┌──────────┐
              │ METRICS  │  │  PLOTS   │  │  BATCH   │
              │ RMSE,    │  │ Traj,    │  │  RUNNER  │
              │ Collision│  │ Error    │  │  Multi   │
              │ Smooth   │  │          │  │  Config  │
              └──────────┘  └──────────┘  └──────────┘
```

## 1.4 Các module trong repo

| Module | Vị trí | Chịu trách nhiệm |
|--------|--------|-------------------|
| **Planner** | `planner/` | Tìm đường A*, sinh trajectory |
| **Controller** | `controller/` | Mô hình xe, điều khiển MPC |
| **Simulator** | `simulator/` | Mô phỏng 2D, collision detection |
| **Evaluation** | `evaluation/` | Metrics, batch runner, plots |
| **MCP Server** | `mcp_server/` | Tool server cho AI Agent |
| **Agent** | `agent/` | Tối ưu config MPC tự động |

---

# Part 2: Grid Map và Biểu diễn môi trường

## 2.1 Grid Map là gì?

Grid map chia môi trường 2D thành các ô vuông đều nhau. Mỗi ô có trạng thái:
- `0` = ô trống (robot đi qua được)
- `1` = vật cản (robot không được đi vào)

```
Grid 8x8, resolution = 0.5m/cell

     0   1   2   3   4   5   6   7
   ┌───┬───┬───┬───┬───┬───┬───┬───┐
0  │   │   │   │   │   │   │   │   │   row 0
   ├───┼───┼───┼───┼───┼───┼───┼───┤
1  │   │   │   │ █ │ █ │   │   │   │   row 1 (vật cản tại col 3,4)
   ├───┼───┼───┼───┼───┼───┼───┼───┤
2  │   │   │   │ █ │ █ │   │   │   │   row 2
   ├───┼───┼───┼───┼───┼───┼───┼───┤
3  │   │   │   │   │   │   │   │   │   row 3
   ├───┼───┼───┼───┼───┼───┼───┼───┤
4  │   │   │   │   │   │   │   │   │   row 4
   ├───┼───┼───┼───┼───┼───┼───┼───┤
5  │   │   │   │   │   │   │   │   │   row 5
   ├───┼───┼───┼───┼───┼───┼───┼───┤
6  │   │   │   │   │   │   │   │   │   row 6
   ├───┼───┼───┼───┼───┼───┼───┼───┤
7  │   │   │   │   │   │   │   │   │   row 7
   └───┴───┴───┴───┴───┴───┴───┴───┘

grid[1][3] = 1  (vật cản)
grid[0][0] = 0  (ô trống)
```

## 2.2 File YAML cấu trúc map

```yaml
name: "obstacles_8x8"
width: 8          # số cột
height: 8         # số hàng
resolution: 0.5   # mét mỗi cell
origin: [0, 0]    # tọa độ thế giới của cell (0,0)
obstacles:
  - [1, 3]        # vật cản tại (row=1, col=3)
  - [1, 4]        # vật cản tại (row=1, col=4)
  - [2, 3]        # vật cản tại (row=2, col=3)
  - [2, 4]        # vật cản tại (row=2, col=4)
```

## 2.3 Tọa độ Cell vs Tọa độ Thế giới

```
Tọa độ cell (row, col)    Tọa độ thế giới (x, y)
┌───┬───┬───┬───┐         ┌───────┬───────┬───────┬───────┐
│0,0│0,1│0,2│0,3│         │ 0,0   │ 0.5,0 │ 1.0,0 │ 1.5,0 │
├───┼───┼───┼───┤         ├───────┼───────┼───────┼───────┤
│1,0│1,1│1,2│1,3│   ───▶  │ 0,0.5 │0.5,0.5│1.0,0.5│1.5,0.5│
├───┼───┼───┼───┤         ├───────┼───────┼───────┼───────┤
│2,0│2,1│2,2│2,3│         │ 0,1.0 │0.5,1.0│1.0,1.0│1.5,1.0│
└───┴───┴───┴───┘         └───────┴───────┴───────┴───────┘

Công thức chuyển đổi:
  x = origin_x + col * resolution
  y = origin_y + row * resolution

Ví dụ: cell (2, 3), resolution=0.5, origin=(0,0)
  x = 0 + 3 * 0.5 = 1.5
  y = 0 + 2 * 0.5 = 1.0
  → world coords = (1.5, 1.0)
```

## 2.4 Collision Detection

```
Kiểm tra va chạm tại điểm (x, y) với bán kính r:

   ┌─────────────────────────────┐
   │  . . . . . . . . . . . . . │
   │  . . . . . . . . . . . . . │
   │  . . . . ┌─────┐ . . . . . │
   │  . . . . │robot│ . . . . . │  ← robot tại (x,y), bán kính r
   │  . . . . │  ●  │ . . . . . │
   │  . . . . └─────┘ . . . . . │
   │  . . . . . . . . . . . . . │
   │  . . . █ █ █ █ █ . . . . . │  ← vật cản
   │  . . . . . . . . . . . . . │
   └─────────────────────────────┘

   Nếu bất kỳ cell nào trong vùng [x-r, x+r] × [y-r, y+r]
   là vật cản → collision = True
```

**Code tương ứng** (`simulator/environment.py`):

```python
def check_collision(self, x: float, y: float, radius: float = 0.0) -> bool:
    r, c = self.world_to_grid(x, y)
    if r < 0 or r >= self.height or c < 0 or c >= self.width:
        return True  # Out of bounds
    if self.grid[r, c] == 1:
        return True  # Direct collision
    # Check neighbors within radius
    ...
```

## 2.5 Bài tập Part 2

- [ ] Mở `maps/obstacles_20x20.yaml`, vẽ ra giấy grid map 20x20 với các vật cản
- [ ] Tính tọa độ thế giới của cell (10, 15) với resolution=0.5
- [ ] Viết code kiểm tra cell (5, 5) có phải vật cản không

---

# Part 3: Thuật toán A* — Tìm đường ngắn nhất

## 3.1 A* là gì?

A* là thuật toán tìm đường trên đồ thị/graf. Kết hợp:
- **g(n)**: chi phí thực tế từ start đến node n
- **h(n)**: heuristic (ước lượng chi phí từ n đến goal)
- **f(n) = g(n) + h(n)**: tổng chi phí ước lượng

A* luôn tìm được đường đi ngắn nhất nếu heuristic là **admissible** (không bao giờ ước lượng quá).

## 3.2 Các bước thuật toán

```
START: (0,0)    GOAL: (3,3)    Grid 4x4

Bước 1: Khởi tạo
┌───┬───┬───┬───┐
│ S │   │   │   │   S = Start (0,0)
├───┼───┼───┼───┤   G = Goal (3,3)
│   │   │   │   │   open_set = {S}
├───┼───┼───┼───┤
│   │   │   │   │
├───┼───┼───┼───┤
│   │   │   │ G │
└───┴───┴───┴───┘

Bước 2: Mở node S, thêm neighbors
┌───┬───┬───┬───┐
│ S │ ① │   │   │   ① = (0,1), g=1, h=4.24, f=5.24
├───┼───┼───┼───┤   ② = (1,0), g=1, h=4.24, f=5.24
│ ② │ ③ │   │   │   ③ = (1,1), g=1.41, h=2.83, f=4.24
├───┼───┼───┼───┤
│   │   │   │   │   open_set = {①, ②, ③}
├───┼───┼───┼───┤   Chọn ③ (f thấp nhất)
│   │   │   │ G │
└───┴───┴───┴───┘

Bước 3: Mở ③, tiếp tục...
┌───┬───┬───┬───┐
│ S │   │   │   │
├───┼───┼───┼───┤
│   │ ③ │ ④ │   │   ④ = (1,2), g=2.41, h=2.24, f=4.65
├───┼───┼───┼───┤   ⑤ = (2,1), g=2.41, h=2.24, f=4.65
│   │ ⑤ │ ⑥ │   │   ⑥ = (2,2), g=2.83, h=1.41, f=4.24
├───┼───┼───┼───┤
│   │   │   │ G │   open_set = {①, ②, ④, ⑤, ⑥}
└───┴───┴───┴───┘   Chọn ⑥ (f thấp nhất)

... tiếp tục cho đến khi goal được mở
```

## 3.3 Pseudocode A*

```python
function A_STAR(grid, start, goal):
    open_set = PriorityQueue()
    open_set.put((0, start))           # (f_score, node)
    came_from = {}                      # để reconstruct path
    g_score = {start: 0}               # chi phí từ start
    f_score = {start: heuristic(start, goal)}

    while open_set is not empty:
        current = open_set.get()        # node có f thấp nhất

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in get_neighbors(current, grid):
            tentative_g = g_score[current] + distance(current, neighbor)

            if tentative_g < g_score.get(neighbor, INFINITY):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                open_set.put((f_score[neighbor], neighbor))

    return NO_PATH
```

## 3.4 Heuristic Functions

```
1. Euclidean Distance (dùng trong repo này):
   h(a, b) = sqrt((a.x - b.x)² + (a.y - b.y)²)

   ┌─────────────────────┐
   │         ● B         │
   │        /            │
   │       /             │
   │      / h = sqrt(8)  │
   │     /               │
   │    /                │
   │   ● A               │
   └─────────────────────┘
   A = (0,0), B = (2,2)
   h = sqrt(4 + 4) = 2.83

2. Manhattan Distance (cho 4-connected):
   h(a, b) = |a.x - b.x| + |a.y - b.y|
   h = |2| + |2| = 4

3. Diagonal Distance (cho 8-connected):
   h(a, b) = max(|a.x - b.x|, |a.y - b.y|)
   h = max(2, 2) = 2
```

## 3.5 4-connected vs 8-connected

```
4-connected (chỉ ngang/dọc):        8-connected (thêm chéo):
     ┌───┬───┬───┐                       ┌───┬───┬───┐
     │   │ ↑ │   │                       │↖ │ ↑ │ ↗│
     ├───┼───┼───┤                       ├───┼───┼───┤
     │ ← │ ● │ → │                       │ ← │ ● │ → │
     ├───┼───┼───┤                       ├───┼───┼───┤
     │   │ ↓ │   │                       │↙ │ ↓ │ ↘│
     └───┴───┴───┘                       └───┴───┴───┘

8-connected cho path ngắn hơn nhưng cần kiểm tra corner-cutting:

Corner-cutting (cắt góc qua vật cản):
     ┌───┬───┐
     │   │ █ │   ← Không được đi chéo nếu cả 2 ô
     ├───┼───┤      bên đều là vật cản
     │ █ │ ● │
     └───┴───┘
```

## 3.6 Code A* trong repo

```python
# planner/astar.py
def astar(grid, start, goal, allow_diagonal=True):
    """A* search on 2D grid."""
    open_set = []
    heapq.heappush(open_set, (0, 0, start))  # (f, counter, node)
    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for dr, dc in directions:
            neighbor = (current[0] + dr, current[1] + dc)
            if not in_bounds(neighbor) or grid[neighbor] == 1:
                continue

            move_cost = sqrt(2) if abs(dr) + abs(dc) == 2 else 1.0
            tentative_g = g_score[current] + move_cost

            if tentative_g < g_score.get(neighbor, inf):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f, counter, neighbor))

    return None  # No path
```

## 3.7 Độ phức tạp

| Metric | Giá trị |
|--------|---------|
| Time complexity | O(b^d) với b=branching factor, d=depth |
| Space complexity | O(b^d) (lưu tất cả nodes đã khám phá) |
| Optimal | Có (với heuristic admissible) |
| Complete | Có (sẽ tìm được path nếu tồn tại) |

## 3.8 Bài tập Part 3

- [ ] Vẽ A* trên giấy với grid 5x5, có 1 vật cản ở giữa
- [ ] Chạy `notebooks/01_path_planning.ipynb`, thay đổi start/goal
- [ ] So sánh path length: 4-connected vs 8-connected
- [ ] Thêm vật cản mới, quan sát A* tìm đường khác

---

# Part 4: Trajectory Generation và Smoothing

## 4.1 Từ Path đến Trajectory

Path A* là list cell rời rạc. Trajectory là chuỗi điểm liên tục theo thời gian:

```
PATH (discrete):                 TRAJECTORY (continuous):
┌───┬───┬───┬───┬───┐
│ ● │ ● │   │   │   │           t=0.0s: (0.0, 0.0), yaw=0.79, v=1.0
├───┼───┼───┼───┼───┤           t=0.1s: (0.1, 0.1), yaw=0.79, v=1.0
│   │   │ ● │   │   │           t=0.2s: (0.2, 0.2), yaw=0.79, v=1.0
├───┼───┼───┼───┼───┤           t=0.3s: (0.3, 0.3), yaw=0.79, v=1.0
│   │   │   │ ● │   │           ...
├───┼───┼───┼───┼───┤           t=0.5s: (0.5, 0.5), yaw=0.79, v=1.0
│   │   │   │   │ ● │
└───┴───┴───┴───┴───┘

[(0,0),(1,1),(2,2),(3,3),(4,4)]  →  DataFrame(t, x, y, yaw, v)
```

## 4.2 Tính Yaw (Heading)

Yaw là góc hướng di động, tính từ trục x dương, ngược chiều kim đồng hồ:

```
                    y
                    ▲
                    │
                    │   ● yaw = π/2 (90°)
                    │  /
                    │ / 
                    │/____________▶ x
                   /
                  /  yaw = 0 (đi sang phải)
                 ●

Công thức:
  yaw = atan2(dy, dx)
  yaw = atan2(y[i+1] - y[i], x[i+1] - x[i])

Ví dụ:
  Điểm A = (0, 0), Điểm B = (1, 1)
  dx = 1, dy = 1
  yaw = atan2(1, 1) = π/4 ≈ 0.785 rad ≈ 45°
```

## 4.3 Interpolation (Nội suy)

Vì path chỉ có vài điểm nhưng MPC cần nhiều điểm theo thời gian, ta nội suy:

```
Path gốc (5 points):     Sau interpolation (20 points):
    ●                        
    │                        ● ● ●
    ●                        │
    │                        ● ● ●
    ●                        │
    │                        ● ● ●
    ●                        │
                             ● ● ●
                             │
                             ●

Dùng scipy.interpolate.interp1d:
  - Nội suy theo arc length (chiều dài cung)
  - Đảm bảo tốc độ đều (constant speed)
```

## 4.4 Smoothing (Làm mượt)

Moving average window giúp giảm gồ ghề:

```
Trước smoothing:              Sau smoothing (window=5):
    ●                           ●
   / \                         / \
  /   ●                       /   \
 /   / \                     /     \
●   /   ●                   ●       ●
 \ /   /                     \     /
  ●   /                       \   /
   \ /                         \ /
    ●                           ●

Công thức:
  y_smooth[i] = (y[i-2] + y[i-1] + y[i] + y[i+1] + y[i+2]) / 5

Lưu ý: Giữ nguyên điểm đầu và cuối.
```

## 4.5 Bài tập Part 4

- [ ] Chạy notebook 01, quan sát trajectory trước/sau smoothing
- [ ] Thay đổi `target_speed` (0.5, 1.0, 2.0), quan sát số điểm trajectory
- [ ] Thay đổi `window` (3, 5, 7, 11), quan sát độ mượt
- [ ] Tính thủ công yaw cho 3 điểm: (0,0), (1,0), (1,1)

---

# Part 5: Mô hình động học xe — Kinematic Bicycle Model

## 5.1 Bicycle Model là gì?

Mô hình bicycle model đơn giản hóa xe 4 bánh thành xe 2 bánh (giống xe đạp):

```
                    FRONT WHEEL (bánh trước)
                         ╱│
                        ╱ │
                       ╱  │ L (wheelbase)
                      ╱   │
                     ╱    │
                    ╱ θ   │
    REAR WHEEL ───●───────┘
    (bánh sau)     │
                   │
                   ▼
              Hướng di chuyển

State: [x, y, yaw, v]
  - x, y: vị trí (tâm bánh sau)
  - yaw: góc hướng (radians)
  - v: vận tốc (m/s)

Control: [steer, accel]
  - steer: góc lái bánh trước (radians)
  - accel: gia tốc (m/s²)
```

## 5.2 Phương trình động học

```
Continuous-time dynamics:
  ẋ = v · cos(θ)
  y = v · sin(θ)
  θ̇ = (v / L) · tan(δ)        ← δ là góc lái
  v̇ = a                         ← a là gia tốc

Discrete-time (Euler forward):
  x[t+1]   = x[t] + v[t] · cos(θ[t]) · dt
  y[t+1]   = y[t] + v[t] · sin(θ[t]) · dt
  θ[t+1]   = θ[t] + (v[t] / L) · tan(δ[t]) · dt
  v[t+1]   = v[t] + a[t] · dt

Ví dụ dt=0.1s, v=1.0m/s, θ=0 (hướng phải), δ=0 (thẳng):
  x[t+1] = x[t] + 1.0 * cos(0) * 0.1 = x[t] + 0.1
  y[t+1] = y[t] + 1.0 * sin(0) * 0.1 = y[t]
  θ[t+1] = 0
  v[t+1] = 1.0
  → Xe đi sang phải 0.1m
```

## 5.3 Trực quan hóa chuyển động

```
steer=0 (đi thẳng):          steer>0 (rẽ phải):
  ──────────────────▶           ──────┐
  x                                ╱   ╲
                                   ╱     ╲
                                  ●       ●

steer<0 (rẽ trái):           accel>0 (tăng tốc):
       ┌───────────           ────●───●───●───●──▶
      ╱                         slow → fast
     ╱
    ●
```

## 5.4 Linearization (Tuyến tính hóa)

MPC cần mô hình tuyến tính. Ta linearize quanh điểm hoạt động (x₀, u₀):

```
f(x, u) ≈ f(x₀, u₀) + A·(x - x₀) + B·(u - u₀)

Trong đó:
  A = ∂f/∂x |_(x₀,u₀)    (Jacobian theo state)
  B = ∂f/∂u |_(x₀,u₀)    (Jacobian theo control)

A = ┌                                    ┐
    │ 1   0   -v·sin(θ)·dt    cos(θ)·dt │
    │ 0   1    v·cos(θ)·dt    sin(θ)·dt │
    │ 0   0    1               tan(δ)/L·dt│
    │ 0   0    0               1          │
    └                                    ┘

B = ┌                                    ┐
    │ 0                    0              │
    │ 0                    0              │
    │ v/(L·cos²(δ))·dt    0              │
    │ 0                    dt             │
    └                                    ┘
```

## 5.5 Bài tập Part 5

- [ ] Chạy notebook 02, mô phỏng xe đi thẳng 100 bước
- [ ] Cho steer=0.3, mô phỏng 100 bước, vẽ quỹ đạo (đường tròn)
- [ ] Tính thủ công: xe tại (0,0,0,1.0), steer=0.2, accel=0, dt=0.1 → state mới?
- [ ] Thử các giá trị steer khác nhau, quan sát bán kính quay

---

# Part 6: Model Predictive Control (MPC)

## 6.1 MPC là gì?

MPC là phương pháp điều khiển tối ưu:
1. Dự đoán trạng thái tương lai trong N bước (horizon)
2. Tìm chuỗi control tối thiểu cost
3. Chỉ áp dụng control đầu tiên
4. Lặp lại ở bước tiếp theo

```
Thời gian ──────────────────────────────────────▶

         ◀──── Horizon (N steps) ────▶
         ┌─────────────────────────────┐
         │  Predicted trajectory       │
    ─────┤  ○ ○ ○ ○ ○ ○ ○ ○ ○ ○      │
    │    └─────────────────────────────┘
    │
    ▼
  Current state

  MPC giải optimization:
    min Σ cost(x[k], u[k])
    s.t. x[k+1] = A·x[k] + B·u[k]  (dynamics)
         |u[k]| ≤ u_max               (constraints)

  Chỉ lấy u[0], áp dụng cho robot.
  Bước tiếp: lặp lại từ state mới.
```

## 6.2 Hàm Cost Function

```
J = Σ[k=0→N-1] (x[k] - x_ref[k])ᵀ · Q · (x[k] - x_ref[k])
  + Σ[k=0→N-1] u[k]ᵀ · R · u[k]
  + (x[N] - x_ref[N])ᵀ · Q_f · (x[N] - x_ref[N])

Trong đó:
  Q = diag(q_x, q_y, q_yaw, q_v)     ← trọng số state error
  R = diag(r_steer, r_accel)           ← trọng số control effort
  Q_f = 2·Q                            ← terminal cost (gấp đôi)

Ý nghĩa:
  ┌─────────────────────────────────────────────────────────┐
  │  Q lớn  → MPC ưu tiên bám quỹ đạo chính xác           │
  │  R lớn  → MPC ưu tiên control mượt, ít thay đổi       │
  │  Q_f lớn → MPC quan tâm đến đích cuối cùng            │
  └─────────────────────────────────────────────────────────┘

Ví dụ:
  Q = [1.0, 1.0, 0.5, 0.1]  ← vị trí quan trọng nhất
  R = [0.1, 0.1]             ← ít penalize control
  → MPC sẽ track tốt nhưng control có thể "gắt"

  Q = [0.5, 0.5, 0.2, 0.05]
  R = [0.5, 0.5]
  → MPC control mượt hơn nhưng track kém hơn
```

## 6.3 Iterative MPC (Tuyến tính hóa lặp)

```
Vì mô hình xe là phi tuyến, ta cần linearize lặp:

Bước 1: Khởi tạo quỹ đạo danh nghĩa từ reference
  x_nom = reference trajectory
  u_nom = zeros

Bước 2: Linearize quanh x_nom, u_nom
  A[k], B[k], c[k] = linearize(x_nom[k], u_nom[k])

Bước 3: Giải QP (convex) bằng CVXPY
  min Σ (x[k]-ref[k])ᵀQ(x[k]-ref[k]) + u[k]ᵀRu[k]
  s.t. x[k+1] = A[k]x[k] + B[k]u[k] + c[k]
       |u[k]| ≤ u_max

Bước 4: Cập nhật danh nghĩa
  x_nom = x_optimal
  u_nom = u_optimal

Bước 5: Lặp lại Bước 2-4 (thường 3 lần)

Sơ đồ:
  ┌─────────────────────────────────────────────┐
  │                                             │
  │   Reference ──▶ Initial Nominal             │
  │                     │                       │
  │                     ▼                       │
  │              ┌─────────────┐                │
  │              │ Linearize   │◀────┐          │
  │              └──────┬──────┘     │          │
  │                     ▼            │          │
  │              ┌─────────────┐     │ Repeat   │
  │              │ Solve QP    │     │ 3 times  │
  │              └──────┬──────┘     │          │
  │                     ▼            │          │
  │              ┌─────────────┐     │          │
  │              │ Update      │─────┘          │
  │              │ Nominal     │                │
  │              └──────┬──────┘                │
  │                     ▼                       │
  │              Control u[0]                   │
  │                                             │
  └─────────────────────────────────────────────┘
```

## 6.4 Constraints (Ràng buộc)

```
Ràng buộc đầu vào (control):
  |steer| ≤ max_steer     (ví dụ: 0.5 rad ≈ 28.6°)
  |accel| ≤ max_accel     (ví dụ: 2.0 m/s²)

Ràng buộc trạng thái:
  v_min ≤ v ≤ v_max       (ví dụ: -0.5 ≤ v ≤ 3.0 m/s)

Sơ đồ feasible region:
  steer
    ▲
    │   ┌───────────┐
    │   │           │
    │   │  FEASIBLE │
    │   │  REGION   │
  ──┼───┼───────────┼───▶ accel
    │   │           │
    │   │           │
    │   └───────────┘
    │
```

## 6.5 Prediction Horizon

```
Horizon ngắn (N=5):              Horizon dài (N=20):
  Predict 5 steps                Predict 20 steps
  ┌─────────┐                    ┌─────────────────────────────┐
  │ ○○○○○   │                    │ ○○○○○○○○○○○○○○○○○○○○       │
  └─────────┘                    └─────────────────────────────┘
  Ưu điểm: Nhanh                 Ưu điểm: Nhìn xa hơn
  Nhược: Không thấy xa           Nhược: Chậm hơn, có thể overfit

Lời khuyên: Bắt đầu với N=10, thử N=5, 15, 20 để so sánh.
```

## 6.6 CVXPY — Giải QP

```python
import cvxpy as cp

# Decision variables
x = cp.Variable((N+1, 4))  # states
u = cp.Variable((N, 2))     # controls

# Cost
cost = 0
for k in range(N):
    err = x[k] - ref[k]
    cost += cp.quad_form(err, Q)    # state error
    cost += cp.quad_form(u[k], R)   # control effort
cost += cp.quad_form(x[N] - ref[N], 2*Q)  # terminal

# Constraints
constraints = [x[0] == x0]  # initial state
for k in range(N):
    constraints += [x[k+1] == A[k]@x[k] + B[k]@u[k] + c[k]]
    constraints += [cp.abs(u[k]) <= u_max]

# Solve
prob = cp.Problem(cp.Minimize(cost), constraints)
prob.solve(solver=cp.OSQP)
```

## 6.7 Bài tập Part 6

- [ ] Chạy notebook 02 với horizon=5, 10, 20, so sánh RMSE
- [ ] Thay đổi Q=[2,2,1,0.2] vs Q=[0.5,0.5,0.2,0.05], quan sát
- [ ] Thay đổi R=[0.5,0.5] vs R=[0.01,0.01], quan sát smoothness
- [ ] Vẽ đồ thị steer và accel cho 2 trường hợp Q khác nhau

---

# Part 7: Simulation và Evaluation

## 7.1 Vòng lặp Simulation

```
┌─────────────────────────────────────────────────────────────────┐
│                    SIMULATION LOOP                               │
│                                                                 │
│  for step in range(max_steps):                                  │
│                                                                 │
│    ┌──────────────┐                                             │
│    │ Current State │──── (x, y, yaw, v)                        │
│    └──────┬───────┘                                             │
│           │                                                     │
│           ▼                                                     │
│    ┌──────────────┐     ┌──────────────┐                       │
│    │ Get Ref      │────▶│ MPC Solve    │                       │
│    │ Window       │     │              │                       │
│    └──────────────┘     └──────┬───────┘                       │
│                                │                                │
│                                ▼                                │
│                         ┌──────────────┐                        │
│                         │ control =    │                        │
│                         │ (steer,accel)│                        │
│                         └──────┬───────┘                        │
│                                │                                │
│                                ▼                                │
│    ┌──────────────┐     ┌──────────────┐                       │
│    │ Collision    │◀────│ Vehicle Step │                       │
│    │ Check        │     │              │                       │
│    └──────┬───────┘     └──────────────┘                       │
│           │                                                     │
│           ▼                                                     │
│    ┌──────────────┐                                             │
│    │ Log Entry    │──── (t, x, y, yaw, v, steer, accel, ...)  │
│    └──────────────┘                                             │
│                                                                 │
│    if goal_reached: break                                       │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## 7.2 Metrics (Chỉ số đánh giá)

```
1. Position RMSE (Root Mean Square Error):
   ┌─────────────────────────────────────────────────┐
   │                                                 │
   │  RMSE = sqrt(1/N * Σ((x[i]-x_ref[i])²          │
   │                      + (y[i]-y_ref[i])²))       │
   │                                                 │
   │  Ý nghĩa: Sai số vị trí trung bình              │
   │  Tốt: < 0.1m   Chấp nhận: < 0.5m   Kém: > 1m   │
   │                                                 │
   └─────────────────────────────────────────────────┘

2. Yaw RMSE:
   RMSE_yaw = sqrt(1/N * Σ(yaw[i] - yaw_ref[i])²)
   Ý nghĩa: Sai số hướng, ảnh hưởng đến path tracking

3. Collision Count:
   Số bước mà robot va chạm vật cản
   Mục tiêu: 0 (luôn luôn)

4. Min Obstacle Distance:
   Khoảng cách nhỏ nhất đến vật cản trong toàn bộ simulation
   Tốt: > 0.5m   Nguy hiểm: < 0.2m

5. Control Smoothness:
   smoothness = mean(|steer[i+1] - steer[i]|)
   Ý nghĩa: Độ mượt tín hiệu điều khiển
   Tốt: < 0.05   Chấp nhận: < 0.2
```

## 7.3 So sánh cấu hình

```
┌──────────────────────────────────────────────────────────────────┐
│  Scenario: basic_circle                                          │
├──────────┬──────────┬──────────┬──────────┬──────────────────────┤
│ Config   │ RMSE     │ Yaw RMSE │ Collision│ Smoothness           │
├──────────┼──────────┼──────────┼──────────┼──────────────────────┤
│ baseline │ 0.0523   │ 0.0312   │ 0        │ 0.0156               │
│ conserv. │ 0.0891   │ 0.0534   │ 0        │ 0.0089  ← mượt nhất │
│ aggress. │ 0.0234   │ 0.0156   │ 0        │ 0.0312  ← RMSE thấp │
│ balanced │ 0.0456   │ 0.0278   │ 0        │ 0.0178               │
└──────────┴──────────┴──────────┴──────────┴──────────────────────┘

Nhận xét:
- Aggressive: RMSE thấp nhất nhưng control "gắt" hơn
- Conservative: Mượt nhất nhưng RMSE cao hơn
- Balanced: Cân bằng giữa RMSE và smoothness
```

## 7.4 Batch Runner

```python
# evaluation/batch_runner.py
def run_batch(scenarios_path, configs_path):
    """Chạy tất cả scenario × config variant."""
    for scenario in scenarios:
        for config in configs:
            result = run_single(scenario, config)
            results.append(result)
    return DataFrame(results)
```

## 7.5 Bài tập Part 7

- [ ] Chạy `python run_pipeline.py --batch --save-results`
- [ ] Mở `data/results/batch_results.csv`, phân tích
- [ ] Thêm config mới vào `configs/mpc_tuning.yaml`, chạy lại batch
- [ ] Vẽ đồ thị so sánh RMSE giữa các config

---

# Part 8: AI Agent và MCP

## 8.1 MCP là gì?

Model Context Protocol (MCP) là giao thức chuẩn để AI Agent gọi tools bên ngoài:

```
┌─────────────────────────────────────────────────────────────────┐
│                    MCP ARCHITECTURE                              │
│                                                                 │
│  ┌──────────────┐         ┌──────────────┐                     │
│  │              │         │              │                     │
│  │  AI AGENT    │◀───────▶│  MCP SERVER  │                     │
│  │              │  MCP    │              │                     │
│  │  - Goal      │ Protocol│  - Register  │                     │
│  │  - Strategy  │         │  - Dispatch  │                     │
│  │  - History   │         │  - Validate  │                     │
│  └──────────────┘         └──────┬───────┘                     │
│                                  │                              │
│                    ┌─────────────┼─────────────┐                │
│                    ▼             ▼             ▼                │
│              ┌──────────┐ ┌──────────┐ ┌──────────┐            │
│              │ list_    │ │ run_mpc_ │ │ evaluate_│            │
│              │ scenarios│ │ sim      │ │ experiment│           │
│              └──────────┘ └──────────┘ └──────────┘            │
│                                                                 │
│  Tool Schema:                                                   │
│  {                                                              │
│    "name": "run_mpc_sim",                                      │
│    "description": "Run MPC simulation with given config",       │
│    "parameters": {                                              │
│      "config": "dict",                                          │
│      "scenario_name": "str"                                     │
│    }                                                            │
│  }                                                              │
└─────────────────────────────────────────────────────────────────┘
```

## 8.2 Safety Validation

```
Mọi config MPC phải vượt qua validation TRƯỚC KHI chạy:

┌─────────────────────────────────────────────────────────────────┐
│                    SAFETY VALIDATION                             │
│                                                                 │
│  Config: {horizon: 10, Q: [1,1,0.5,0.1], R: [0.1,0.1], ...}  │
│                         │                                       │
│                         ▼                                       │
│              ┌─────────────────────┐                            │
│              │ validate_config()   │                            │
│              └──────────┬──────────┘                            │
│                         │                                       │
│         ┌───────────────┼───────────────┐                      │
│         ▼               ▼               ▼                      │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐              │
│  │ horizon     │ │ Q, R        │ │ max_steer   │              │
│  │ 1 ≤ h ≤ 30 │ │ ≥ 0         │ │ < 1.57 rad  │              │
│  └──────┬──────┘ └──────┬──────┘ └──────┬──────┘              │
│         │               │               │                      │
│         └───────────────┼───────────────┘                      │
│                         ▼                                       │
│              ┌─────────────────────┐                            │
│              │ Valid?              │                            │
│              │ Yes → Run           │                            │
│              │ No  → Return error  │                            │
│              └─────────────────────┘                            │
│                                                                 │
│  Safe Ranges:                                                   │
│  ┌────────────────┬───────────┬───────────┐                    │
│  │ Parameter      │ Min       │ Max       │                    │
│  ├────────────────┼───────────┼───────────┤                    │
│  │ horizon        │ 1         │ 30        │                    │
│  │ dt             │ 0.01      │ 1.0       │                    │
│  │ Q (each)       │ 0.0       │ 100.0     │                    │
│  │ R (each)       │ 0.0       │ 100.0     │                    │
│  │ max_steer      │ 0.01      │ 1.57 rad  │                    │
│  │ max_accel      │ 0.1       │ 10.0      │                    │
│  │ max_speed      │ 0.1       │ 10.0      │                    │
│  └────────────────┴───────────┴───────────┘                    │
└─────────────────────────────────────────────────────────────────┘
```

## 8.3 Agent Loop

```
┌─────────────────────────────────────────────────────────────────┐
│                    AI AGENT LOOP                                 │
│                                                                 │
│  Input: goal = {rmse_target: 0.3, collision_target: 0,         │
│                 max_trials: 10}                                 │
│                                                                 │
│  for trial in range(max_trials):                                │
│                                                                 │
│    ┌─────────────────────────────────────────────────────────┐  │
│    │ Step 1: Suggest Config                                  │  │
│    │                                                         │  │
│    │  if history is empty:                                   │  │
│    │    config = baseline                                    │  │
│    │  else:                                                  │  │
│    │    last = history[-1]                                   │  │
│    │    if last.collision > 0:                               │  │
│    │      config.R *= 1.5      ← tăng penalty                │  │
│    │      config.speed *= 0.8  ← giảm tốc                   │  │
│    │    if last.rmse > target * 1.5:                         │  │
│    │      config.horizon += 2  ← tăng horizon                │  │
│    │      config.Q *= 1.2      ← tăng tracking weight        │  │
│    │    config += random_perturbation()                      │  │
│    └─────────────────────────────────────────────────────────┘  │
│                         │                                       │
│                         ▼                                       │
│    ┌─────────────────────────────────────────────────────────┐  │
│    │ Step 2: Validate Config                                 │  │
│    │   valid, reason = server.call("validate_config", config)│  │
│    │   if not valid: skip trial                              │  │
│    └─────────────────────────────────────────────────────────┘  │
│                         │                                       │
│                         ▼                                       │
│    ┌─────────────────────────────────────────────────────────┐  │
│    │ Step 3: Run Simulation                                  │  │
│    │   result = server.call("run_mpc_sim", config=config)    │  │
│    │   metrics = result["metrics"]                           │  │
│    └─────────────────────────────────────────────────────────┘  │
│                         │                                       │
│                         ▼                                       │
│    ┌─────────────────────────────────────────────────────────┐  │
│    │ Step 4: Update Best                                     │  │
│    │   if metrics.collision < best.collision:                │  │
│    │     best = {config, metrics}                            │  │
│    │   elif metrics.rmse < best.rmse:                        │  │
│    │     best = {config, metrics}                            │  │
│    └─────────────────────────────────────────────────────────┘  │
│                         │                                       │
│                         ▼                                       │
│    ┌─────────────────────────────────────────────────────────┐  │
│    │ Step 5: Check Goal                                      │  │
│    │   if rmse ≤ target and collision ≤ target:              │  │
│    │     break  ← Goal reached!                              │  │
│    └─────────────────────────────────────────────────────────┘  │
│                                                                 │
│  Output: best_config, best_metrics, history                     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## 8.4 Mở rộng: LLM Integration

```
Hiện tại: Rule-based strategy (không cần LLM)

Mở rộng: Dùng LLM để suggest config

┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│  ┌──────────┐    ┌──────────────┐    ┌──────────────┐          │
│  │          │    │              │    │              │          │
│  │  PROMPT  │───▶│    LLM       │───▶│  Parse JSON  │          │
│  │          │    │  (GPT,       │    │  config      │          │
│  │ "Goal:   │    │   Claude)    │    │              │          │
│  │  RMSE<0.3│    │              │    └──────┬───────┘          │
│  │  History:│    │              │           │                  │
│  │  ..."    │    │              │           ▼                  │
│  └──────────┘    └──────────────┘    ┌──────────────┐          │
│                                      │ Validate &   │          │
│                                      │ Run          │          │
│                                      └──────────────┘          │
│                                                                 │
│  Prompt template: agent/prompts.py                              │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## 8.5 Bài tập Part 8

- [ ] Chạy notebook 05, hiểu flow agent
- [ ] Thêm tool mới `get_scenario_info` vào MCP server
- [ ] Thử goal khác: `collision_target: 0, rmse_target: 0.5`
- [ ] Viết strategy mới trong `agent/prompts.py` (random search có giới hạn)
- [ ] (Nâng cao) Tích hợp LLM API để suggest config

---

# Tài liệu tham khảo đầy đủ

## MPC và Control
1. Rawlings, J.B., Mayne, D.Q., Diehl, M. — *Model Predictive Control: Theory, Computation, and Design*. https://sites.engineering.ucsb.edu/~jbraw/mpc/
2. mpc_python Repository — https://github.com/mcarfagno/mpc_python
3. CVXPY Documentation — https://www.cvxpy.org/
4. CVXPY Examples — https://www.cvxpy.org/examples/
5. pyMPC — http://www.marcoforgione.it/pyMPC/

## Path Planning
6. Hart, P.E., Nilsson, N.J., Raphael, B. — *A Formal Basis for the Heuristic Determination of Minimum Cost Paths* (1968). https://doi.org/10.1109/TSSC.1968.300136
7. PythonRobotics — https://github.com/AtsushiSakai/PythonRobotics
8. PythonRobotics A* — https://atsushisakai.github.io/PythonRobotics/modules/5_path_planning/grid_base_search/grid_base_search.html
9. LaValle, S.M. — *Planning Algorithms*. https://lavalle.pl/planning/

## Robot mô phỏng
10. MuJoCo Documentation — https://mujoco.readthedocs.io/
11. MuJoCo GitHub — https://github.com/google-deepmind/mujoco
12. MuSHR — https://mushr.io/
13. PyBullet — https://pybullet.org/wordpress/

## AI Agents và MCP
14. Model Context Protocol — https://modelcontextprotocol.io/
15. MCP GitHub — https://github.com/modelcontextprotocol
16. Anthropic: Introduction to MCP — https://www.anthropic.com/news/model-context-protocol
17. Hugging Face Agents Course — https://huggingface.co/learn/agents-course
18. Coursera: AI Agents with MCP — https://www.coursera.org/specializations/ai-agents-model-context-protocol
19. IBM: Build AI Agents using MCP — https://www.coursera.org/learn/build-ai-agents-using-mcp
20. Kaggle: 5-Day AI Agents — https://www.kaggle.com/learn-guide/5-day-agents

## Mathematics Background
21. Boyd, S., Vandenberghe, L. — *Convex Optimization*. https://web.stanford.edu/~boyd/cvxbook/
22. Nocedal, J., Wright, S.J. — *Numerical Optimization*. https://www.springer.com/gp/book/9780387303031

---

# Glossary (Thuật ngữ)

| Thuật ngữ | Giải thích |
|-----------|------------|
| **A*** | Thuật toán tìm đường ngắn nhất trên đồ thị, sử dụng heuristic |
| **Bicycle Model** | Mô hình động học đơn giản hóa xe 4 bánh thành 2 bánh |
| **Collision Detection** | Kiểm tra va chạm giữa robot và vật cản |
| **CVXPY** | Python library giải bài toán tối ưu lồi (convex optimization) |
| **Grid Map** | Biểu diễn môi trường 2D bằng ma trận ô vuông |
| **Heuristic** | Hàm ước lượng chi phí, dùng trong A* |
| **Horizon (N)** | Số bước dự đoán tương lai trong MPC |
| **Iterative MPC** | MPC với tuyến tính hóa lặp (linearize → solve → update) |
| **MCP** | Model Context Protocol — giao thức AI Agent gọi tools |
| **MPC** | Model Predictive Control — phương pháp điều khiển tối ưu dự đoán |
| **QP** | Quadratic Programming — bài toán tối ưu bậc 2 |
| **RMSE** | Root Mean Square Error — căn bậc hai của trung bình bình phương sai số |
| **Smoothing** | Làm mượt trajectory bằng moving average |
| **State** | Trạng thái robot: [x, y, yaw, v] |
| **Steer** | Góc lái bánh trước (radians) |
| **Trajectory** | Chuỗi điểm theo thời gian: (t, x, y, yaw, v) |
| **Yaw** | Góc hướng robot (radians, từ trục x dương) |
