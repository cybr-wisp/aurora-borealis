#!/usr/bin/env bash
set -euo pipefail

repo_root="${1:-.}"
cd "$repo_root"

printf 'Aurora Borealis Day 1-4 validation\n'
python3 -m pip install -r requirements.txt
python3 -m pytest -q simulation/tests
python3 experiments/day1_4_metrics.py

if command -v cmake >/dev/null 2>&1 && command -v protoc >/dev/null 2>&1; then
  cmake -S engine -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
  cmake --build build
  ctest --test-dir build --output-on-failure
  ./build/bench_ingestion
else
  printf 'C++ toolchain not fully installed. On Ubuntu/Debian run:\n'
  printf 'sudo apt-get update && sudo apt-get install -y cmake ninja-build g++ libeigen3-dev libprotobuf-dev protobuf-compiler libgtest-dev\n'
fi
