# Cyber Planner 2025

Team 8214's 2025 FRC season code for superstructure motion planning, written in CPP and Eigen Lib. See our math work and code report [here](./report.pdf).

![](./assets/next-innovation.png)

[![Next-Innovation](https://img.shields.io/badge/Next-Innovation-blueviolet?style=flat)](https://github.com/FRCNextInnovation) [![Lang](https://img.shields.io/badge/Lang-en--US-Green?style=flat)]()

The algorithm is based on a finite convex polygon set representation of objects, where path planning is built upon a Lipschitz continuous artificial potential field. In trajectory planning, constraints on current and voltage are incorporated, and time-optimal trajectories are derived using the Augmented Lagrangian method and the L-BFGS optimization algorithm.

![](./assets/trajectory.png)

## Quick Start

### Environment Setup

- C++17 or higher
- CMake 3.15 or higher

### Linux / macOS

```bash
bash scripts/build.sh

./build/server
```

```bash
./build/client
```

### Windows

```bash
.\scripts\build.ps.1

.\build\Release\server.exe
```

```bash
.\build\Release\client.exe
```

## See Also

1. Z. Zirui, CoTiMo. (2024). [Online]. Available: https://github.com/ZhangzrJerry/CoTiMo
1. D. Verscheure, B. Demeulenaere, J. Swevers, J. De Schutter, and M. Diehl, "Time-Optimal Path Tracking for Robots: A Convex Optimization Approach," IEEE Trans. Automat. Contr., vol. 54, no. 10, pp. 2318–2327, Oct. 2009, doi: 10.1109/TAC.2009.2028959.