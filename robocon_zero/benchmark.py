#!/usr/bin/env python3
import argparse, json, pathlib, sys
print(">>> 进入 main", flush=True)            # 冲刷缓冲
from bench.runner import BenchmarkRunner
print(">>> 已导入", flush=True)

ap = argparse.ArgumentParser()
ap.add_argument("--track", default="bench/tracks/mini_4p.json")
ap.add_argument("--policy", default="simple")
ap.add_argument("--dump-trace", type=pathlib.Path)
args = ap.parse_args()

print(">>> 实例化 BenchmarkRunner …", flush=True)
runner = BenchmarkRunner(args.track)
print(">>> 开始 run …", flush=True)
stat = runner.run(args.policy)
print(">>> run 返回", flush=True)

print(json.dumps(stat.__dict__, indent=2, ensure_ascii=False))
if args.dump_trace:
    args.dump_trace.write_text(json.dumps(stat.trace, indent=2, ensure_ascii=False))
print(">>> 写完文件", flush=True)
