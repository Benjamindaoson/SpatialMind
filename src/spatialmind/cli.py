"""Command-line runner, benchmark and evidence inspection."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from spatialmind.benchmark import run_suite
from spatialmind.runtime import open_runtime


def main() -> None:
    parser = argparse.ArgumentParser(prog="spatialmind")
    commands = parser.add_subparsers(dest="command", required=True)
    demo = commands.add_parser("demo", help="Execute a grounded mobile robot task")
    demo.add_argument("--instruction", default="Find the blue toolbox")
    demo.add_argument("--llm-model", default=None, help="Opt-in OpenAI-compatible model")
    demo.add_argument("--llm-base-url", default="https://api.openai.com/v1")
    demo.add_argument("--data-dir", default=".spatialmind")
    demo.add_argument("--max-actions", type=int, default=80)
    demo.add_argument("--resume", help="Resume an existing task id")
    demo.add_argument("--move-toolbox", choices=["office", "storage"])
    demo.add_argument("--obstacle", action="store_true", help="Block east doorway")
    demo.add_argument("--show-world", action="store_true")
    bench = commands.add_parser("benchmark", help="Run interventions and ablations")
    bench.add_argument("--output", default="artifacts/benchmark.json")
    bench.add_argument("--seeds", type=int, default=1, help="Number of independent deterministic seeds")
    bench.add_argument("--trace-dir", default=None, help="Save event logs for every trial")
    metric = commands.add_parser("physical-benchmark", help="Paired metric-robot interventions")
    metric.add_argument("--seeds", type=int, default=3)
    metric.add_argument("--output-dir", default="artifacts/metric-benchmark")
    metric.add_argument("--trace", action="store_true")
    report = commands.add_parser("trace", help="Inspect a task's event stream")
    report.add_argument("task_id")
    report.add_argument("--data-dir", default=".spatialmind")
    args = parser.parse_args()

    if args.command == "benchmark":
        print(json.dumps(run_suite(output=args.output, seeds=args.seeds, trace_dir=args.trace_dir), indent=2))
        return
    if args.command == "physical-benchmark":
        from spatialmind.physical_benchmark import run_physical_benchmark
        print(json.dumps(run_physical_benchmark(seeds=args.seeds,
                         output_dir=args.output_dir, trace=args.trace), indent=2))
        return

    interpreter = None
    if args.command == "demo" and args.llm_model:
        from spatialmind.llm import OpenAICompatibleInterpreter
        interpreter = OpenAICompatibleInterpreter(
            model=args.llm_model, base_url=args.llm_base_url,
        )
    runtime, world = open_runtime(data_dir=args.data_dir, interpreter=interpreter)
    if args.command == "trace":
        print(json.dumps(runtime.events.events(args.task_id), indent=2))
        return
    if args.move_toolbox:
        target = world.room_center(args.move_toolbox)
        world.move_object("toolbox_1", target)
    if args.obstacle:
        from spatialmind.models import Point
        world.set_obstacle(Point(9, 3))
    if args.show_world:
        print(world.render(runtime.robot.pose))
    outcome = runtime.run(
        None if args.resume else args.instruction,
        task_id=args.resume,
        resume=bool(args.resume),
        max_actions=args.max_actions,
    )
    print(json.dumps(outcome.to_dict(), indent=2))
    print("Task trace stored in", Path(args.data_dir) / "events.sqlite3")


if __name__ == "__main__":
    main()
