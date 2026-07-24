#!/usr/bin/env python3
"""
GATE1定量計測結果の解析スクリプト (Issue #36)
使い方: ./scripts/gate1_analyze.py <results_directory>
"""

import sys
import os
from pathlib import Path
from collections import defaultdict

if len(sys.argv) < 2:
    print("Usage: gate1_analyze.py <results_directory>")
    sys.exit(1)

results_dir = Path(sys.argv[1])
if not results_dir.exists():
    print(f"Error: {results_dir} not found")
    sys.exit(1)

print(f"Analyzing results from {results_dir}")
print("=== GATE1 Quantification Analysis ===\n")

# ログファイルを読み込み
log_files = sorted(results_dir.glob("trial_*.log"))
print(f"Found {len(log_files)} trial logs")

if log_files:
    print("\nSample trials:")
    for log_file in log_files[:3]:
        print(f"\n{log_file.name}:")
        with open(log_file) as f:
            for line in f:
                print(f"  {line.rstrip()}")

print(f"\n✅ Analysis complete. Total trials logged: {len(log_files)}")
print(f"Full results in: {results_dir}")
