# Glossary — Bảng thuật ngữ

Bảng tra cứu nhanh tất cả thuật ngữ sử dụng trong repo.

---

## A

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **A*** | A-star | Thuật toán tìm đường ngắn nhất trên đồ thị, sử dụng heuristic để hướng dẫn tìm kiếm |
| **Acceleration** | Gia tốc | Tốc độ thay đổi vận tốc, đơn vị m/s² |
| **Admissible** | Hợp lệ | Heuristic không bao giờ ước lượng quá chi phí thực tế |
| **Arc length** | Chiều dài cung | Khoảng cách dọc theo đường cong, không phải khoảng cách thẳng |

## B

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **Bicycle Model** | Mô hình xe đạp | Mô hình động học đơn giản hóa xe 4 bánh thành 2 bánh |
| **Batch Runner** | Trình chạy hàng loạt | Chạy nhiều scenario × config tự động |

## C

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **Collision** | Va chạm | Robot chạm vào vật cản hoặc ra ngoài biên |
| **Constraint** | Ràng buộc | Giới hạn trên state hoặc control trong MPC |
| **Convex** | Lồi | Bài toán tối ưu chỉ có 1 minimum global |
| **Control** | Điều khiển | Tín hiệu đầu ra: [steer, accel] |
| **Cost Function** | Hàm mục tiêu | Hàm cần minimize trong MPC |
| **CVXPY** | — | Python library giải bài toán tối ưu lồi |

## D

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **Decision Variable** | Biến quyết định | Vector x, u mà MPC cần tìm |
| **Discrete Time** | Thời gian rời rạc | Mô hình chỉ xét tại các thời điểm t = 0, dt, 2dt, ... |
| **dt** | Time step | Bước thời gian, thường 0.1s |

## E

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **Euler Forward** | Euler tiến | Phương pháp tích phân số đơn giản: x[t+1] = x[t] + f(x[t])·dt |
| **Euclidean Distance** | Khoảng cách Euclidean | sqrt((x₁-x₂)² + (y₁-y₂)²) |

## F

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **Feasible Region** | Vùng khả thi | Tập tất cả (x,u) thỏa mãn ràng buộc |
| **Fallback Controller** | Bộ điều khiển dự phòng | Controller đơn giản khi MPC không hội tụ |

## G

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **g(n)** | Cost to come | Chi phí thực tế từ start đến node n |
| **Goal** | Đích | Vị trí mà robot cần đến |
| **Grid Map** | Bản đồ lưới | Biểu diễn môi trường 2D bằng ma trận ô vuông |

## H

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **h(n)** | Heuristic | Ước lượng chi phí từ node n đến goal |
| **f(n)** | Total cost | f = g + h, tổng chi phí ước lượng |
| **Heading** | Hướng | Góc yaw của robot |
| **Horizon** | Chân trời dự đoán | Số bước dự đoán tương lai trong MPC (N) |

## I

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **Interpolation** | Nội suy | Tính giá trị trung gian giữa các điểm đã biết |
| **Iterative MPC** | MPC lặp | MPC với tuyến tính hóa lặp: linearize → solve → update |

## J

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **Jacobian** | Ma trận Jacobian | Ma trận đạo hàm riêng, dùng để linearize mô hình phi tuyến |

## K

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **Kinematic** | Động học | Mô tả chuyển động không xét lực |

## L

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **Linearization** | Tuyến tính hóa | Xấp xỉ hàm phi tuyến bằng hàm tuyến tính quanh một điểm |
| **Linear Regression** | Hồi quy tuyến tính | Mô hình y = ax + b |
| **Log** | Nhật ký | Dữ liệu ghi lại trong simulation |

## M

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **MCP** | Model Context Protocol | Giao thức AI Agent gọi tools bên ngoài |
| **MPC** | Model Predictive Control | Phương pháp điều khiển tối ưu dự đoán |
| **Moving Average** | Trung bình động | Lấy trung bình trong cửa sổ trượt |

## N

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **Nominal Trajectory** | Quỹ đạo danh nghĩa | Quỹạo tham chiếu để linearize quanh |

## O

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **Objective Function** | Hàm mục tiêu | Hàm cần minimize/maximize |
| **OSQP** | — | Solver giải QP (Operator Splitting QP) |
| **Optimal** | Tối ưu | Giải tốt nhất trong feasible region |

## P

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **Path** | Đường đi | List các cell từ start đến goal |
| **Prediction** | Dự đoán | Tính state tương lai dựa trên mô hình |
| **Priority Queue** | Hàng đợi ưu tiên | Data structure lấy phần tử có priority thấp nhất |

## Q

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **Q** | State cost matrix | Ma trận trọng số lỗi state: diag(q_x, q_y, q_yaw, q_v) |
| **QP** | Quadratic Programming | Bài toán tối ưu bậc 2 với ràng buộc tuyến tính |
| **Quadratic Form** | Dạng bậc 2 | xᵀQx, tổng trọng số bình phương |

## R

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **R** | Control cost matrix | Ma trận trọng số control: diag(r_steer, r_accel) |
| **Reference** | Tham chiếu | Quỹạo mong muốn robot bám theo |
| **Resolution** | Độ phân giải | Mét mỗi cell trong grid map |
| **RMSE** | Root Mean Square Error | Căn bậc hai của trung bình bình phương sai số |
| **Rolling Window** | Cửa sổ trượt | Tập dữ liệu trong khoảng thời gian trượt |

## S

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **Scenario** | Kịch bản | Tổ hợp map + start + goal + MPC config |
| **SCS** | — | Solver giải QP (Splitting Conic Solver) |
| **Smoothing** | Làm mượt | Giảm gồ ghề trong trajectory |
| **State** | Trạng thái | [x, y, yaw, v] — vị trí, hướng, vận tốc |
| **Steer** | Góc lái | Góc quay bánh trước (radians) |
| **Surrogate** | Mô hình thay thế | ML model thay cho mô hình vật lý |

## T

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **Terminal Cost** | Chi phí cuối cùng | Cost tại bước cuối horizon (thường = 2×Q) |
| **Time Step** | Bước thời gian | dt, khoảng thời gian giữa 2 bước |
| **Trajectory** | Quỹ đạo | Chuỗi điểm theo thời gian: (t, x, y, yaw, v) |

## V

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **Validation** | Kiểm định | Kiểm tra config trước khi chạy |
| **Vehicle Model** | Mô hình xe | Mô tả động học robot |
| **v** | Vận tốc | Tốc độ robot (m/s) |

## W

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **Walk-forward** | Đi tới | Phương pháp validation mở rộng window dần |
| **Wheelbase** | Chiều dài cơ sở | Khoảng cách giữa 2 bánh xe (L) |
| **World Coordinates** | Tọa độ thế giới | Hệ tọa độ chung của môi trường |

## Y

| Thuật ngữ | Tiếng Anh | Giải thích |
|-----------|-----------|------------|
| **Yaw** | Góc hướng | Góc quay quanh trục z (radians, từ trục x dương, ngược chiều kim đồng hồ) |
