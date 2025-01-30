# Cyber Planner 2025

[![Next-Innovation](https://img.shields.io/badge/Next-Innovation-blueviolet?style=flat)](https://github.com/FRCNextInnovation) [![Lang](https://img.shields.io/badge/Lang-en--US-Green?style=flat)]()

Team 8214's 2025 FRC season code for superstructure motion planning, written in GRPC

<img src="./assets/next-innovation.png" style="zoom:50%;" >

## Environment

- C++17 or higher
- CMake 3.15 or higher
- vcpkg package manager

## Quick Start

1. Install vcpkg if you haven't:
    ```bash
    git clone https://github.com/Microsoft/vcpkg.git
    cd vcpkg
    ./bootstrap-vcpkg.sh  # Linux/macOS
    # OR
    .\bootstrap-vcpkg.bat  # Windows
    ```

2. Set VCPKG_ROOT environment variable:
    ```bash
    # Linux/macOS
    export VCPKG_ROOT=/path/to/vcpkg

    # Windows (PowerShell)
    $env:VCPKG_ROOT = "C:\path\to\vcpkg"
    ```

3. Build the project:
    ```bash
    # Linux/macOS
    bash scripts/build.sh

    # Windows
    .\scripts\build.ps.1
    ```

## See Also

1. Z. Zirui, CoTiMo. (2024). [Online]. Available: https://github.com/ZhangzrJerry/CoTiMo
1. W. Zhepei, LBFGS-Lite. (2021). [Online]. Available: https://github.com/ZJU-FAST-Lab/LBFGS-Lite
1. D. Verscheure, B. Demeulenaere, J. Swevers, J. De Schutter, and M. Diehl, "Time-Optimal Path Tracking for Robots: A Convex Optimization Approach," IEEE Trans. Automat. Contr., vol. 54, no. 10, pp. 2318–2327, Oct. 2009, doi: 10.1109/TAC.2009.2028959.