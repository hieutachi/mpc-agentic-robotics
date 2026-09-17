# evaluation — Evaluation Module

Module đánh giá chất lượng bám quỹ đạo, an toàn, và so sánh cấu hình MPC.

## Files

- `metrics.py` — Tính metric: position RMSE, yaw RMSE, completion time, collision count, control smoothness.
- `batch_runner.py` — Chạy batch experiments trên nhiều scenario × config.
- `plots.py` — Vẽ trajectory, error, và biểu đồ so sánh.

## Usage

```python
from evaluation import compute_metrics, run_batch, plot_trajectory

# Single run
metrics = compute_metrics(log_df, ref_df)

# Batch
results = run_batch("configs/scenarios.yaml", "configs/mpc_tuning.yaml")
print(results[["scenario", "config", "position_rmse", "collision_count"]])
```

## Run Tests

```bash
pytest evaluation/tests/ -v
```
