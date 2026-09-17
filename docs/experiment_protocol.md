# Experiment Protocol

Quy trình thí nghiệm chuẩn để đảm bảo kết quả có thể tái lập.

## Bước 1: Chọn scenario

```bash
python run_pipeline.py --scenario basic_circle --save-log --save-plot
```

Kiểm tra:
- [ ] Log CSV được lưu trong `data/logs/`
- [ ] Plot được lưu trong `data/results/`
- [ ] Không có collision trên map trống

## Bước 2: Chạy batch

```bash
python run_pipeline.py --batch --save-results
```

Kiểm tra:
- [ ] Bảng kết quả có 3 scenario × 4 config = 12 dòng
- [ ] Không có lỗi "no_path_found"
- [ ] Kết quả lưu trong `data/results/batch_results.csv`

## Bước 3: So sánh cấu hình

Phân tích `batch_results.csv`:
- Config nào có RMSE thấp nhất?
- Config nào có 0 collision trên tất cả scenario?
- Trade-off giữa RMSE và control smoothness?

## Bước 4: Agent optimization

```bash
python agent/run_agent.py --scenario obstacles --max-trials 10 --save-results data/results/agent_run.json
```

So sánh:
- Agent tìm config tốt hơn baseline không?
- Agent mất bao nhiêu trials để đạt goal?

## Bước 5: Ghi nhận kết quả

Lưu tất cả:
- `data/logs/*.csv` — simulation logs
- `data/results/*.csv` — batch results
- `data/results/*.json` — agent results
- `data/results/*.png` — plots

Ghi chú:
- Version Python: `python --version`
- Version dependencies: `pip freeze > requirements_frozen.txt`
- Ngày chạy thí nghiệm
