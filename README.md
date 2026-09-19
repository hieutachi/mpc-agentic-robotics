# MPC Agentic Robotics

**Khung há»c Model Predictive Control cho robot di Ä‘á»™ng trÃªn Python, káº¿t há»£p AI Agent vÃ  MCP Ä‘á»ƒ tá»± Ä‘á»™ng cáº¥u hÃ¬nh, mÃ´ phá»ng vÃ  Ä‘Ã¡nh giÃ¡ bÃ i toÃ¡n bÃ¡m quá»¹ Ä‘áº¡o, tÃ¬m Ä‘Æ°á»ng vÃ  trÃ¡nh váº­t cáº£n.**

## Káº¿t quáº£ há»c táº­p

Sau khi hoÃ n thÃ nh repo nÃ y, sinh viÃªn sáº½:

- Hiá»ƒu nguyÃªn lÃ½ MPC vÃ  cÃ¡ch triá»ƒn khai báº±ng CVXPY
- Biáº¿t cÃ¡ch tÃ¬m Ä‘Æ°á»ng báº±ng A* vÃ  sinh trajectory tham chiáº¿u
- XÃ¢y dá»±ng pipeline mÃ´ phá»ng robot di Ä‘á»™ng end-to-end
- ÄÃ¡nh giÃ¡ cháº¥t lÆ°á»£ng Ä‘iá»u khiá»ƒn báº±ng metric cÃ³ cáº¥u trÃºc
- Sá»­ dá»¥ng AI Agent káº¿t há»£p MCP Ä‘á»ƒ tá»± Ä‘á»™ng tá»‘i Æ°u cáº¥u hÃ¬nh

## CÃ i Ä‘áº·t nhanh

```bash
# 1. Clone repo
git clone <repo-url>
cd mpc-agentic-robotics

# 2. Táº¡o virtual environment
python -m venv venv
source venv/bin/activate  # Linux/Mac
# venv\Scripts\activate   # Windows

# 3. CÃ i dependencies
pip install -r requirements.txt

# 4. Cháº¡y demo
python run_pipeline.py --scenario basic_circle --save-plot
```

## Cháº¡y demo

```bash
# Single scenario
python run_pipeline.py --scenario basic_circle

# All scenarios Ã— all configs
python run_pipeline.py --batch --save-results

# AI Agent
python agent/run_agent.py --scenario basic_circle --max-trials 5
```

## Cáº¥u trÃºc repo

```
mpc-agentic-robotics/
â”œâ”€â”€ run_pipeline.py          # Script cháº¡y end-to-end
â”œâ”€â”€ configs/                 # MPC config, scenarios
â”œâ”€â”€ maps/                    # Grid map máº«u
â”œâ”€â”€ planner/                 # A* + trajectory generation
â”œâ”€â”€ controller/              # Vehicle model + Iterative MPC (CVXPY)
â”œâ”€â”€ simulator/               # Environment + simulation loop
â”œâ”€â”€ evaluation/              # Metrics + batch runner + plots
â”œâ”€â”€ mcp_server/              # MCP tool server + safety validation
â”œâ”€â”€ agent/                   # AI Agent loop
â”œâ”€â”€ notebooks/               # 5 notebook hÆ°á»›ng dáº«n
â”œâ”€â”€ docs/                    # TÃ i liá»‡u chi tiáº¿t
â””â”€â”€ data/                    # Logs, results
```

## TÃ i liá»‡u

- **[SELF_STUDY.md](SELF_STUDY.md)** â€” TÃ i liá»‡u tá»± há»c chi tiáº¿t (8 pháº§n, sÆ¡ Ä‘á»“ ASCII, cÃ´ng thá»©c toÃ¡n) â­
- **[SELF_STUDY_EXAMPLE.md](SELF_STUDY_EXAMPLE.md)** â€” VÃ­ dá»¥ xuyÃªn suá»‘t vá»›i sá»‘ liá»‡u cá»¥ thá»ƒ â­â­
- **[LEARNING_PATH.md](LEARNING_PATH.md)** â€” ÄÆ°á»ng dáº«n tÃ i liá»‡u há»c theo tuáº§n
- **[ARCHITECTURE.md](ARCHITECTURE.md)** â€” Kiáº¿n trÃºc há»‡ thá»‘ng vÃ  sÆ¡ Ä‘á»“
- **[PROJECT_CHECKLIST.md](PROJECT_CHECKLIST.md)** â€” Checklist cÃ´ng viá»‡c cho nhÃ³m
- **[docs/concepts.md](docs/concepts.md)** â€” Kiáº¿n thá»©c ná»n táº£ng (tá»‘i Æ°u lá»“i, ma tráº­n, Jacobian)
- **[docs/glossary.md](docs/glossary.md)** â€” Báº£ng thuáº­t ngá»¯ A-Z

## PhÃ¢n cÃ´ng nhÃ³m 5 ngÆ°á»i

| ThÃ nh viÃªn | Vai trÃ² | Module chÃ­nh |
|---|---|---|
| Member 1 | NhÃ³m trÆ°á»Ÿng / TÃ­ch há»£p | `run_pipeline.py`, configs, docs |
| Member 2 | Path Planning | `planner/` |
| Member 3 | MPC & Control | `controller/` |
| Member 4 | Simulator & Evaluation | `simulator/`, `evaluation/` |
| Member 5 | AI Agent & MCP | `mcp_server/`, `agent/` |

## Cháº¡y tests

```bash
pytest planner/ controller/ simulator/ evaluation/ mcp_server/ -v
```

## YÃªu cáº§u há»‡ thá»‘ng

- Python 3.9+
- OS: Linux, macOS, Windows
- KhÃ´ng cáº§n GPU, khÃ´ng cáº§n MuJoCo/PyBullet (mÃ´ phá»ng 2D grid-based)
