#!/usr/bin/env python3
"""Run the six-source offline ingestion with explicit availability reporting.

Nothing is downloaded and no source license or account gate is bypassed.
Example:
  python scripts/run_data_pipeline.py --config configs/datasets.example.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# Source checkout and editable-install both supported.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from spatialmind.datasets import DATASETS, prepare_dataset
from spatialmind.datasets.replay import replay_rgbd


def run(config_file: Path, *, selected: set[str] | None = None,
        with_replay: bool = False, fail_on_missing: bool = False):
    data = json.loads(config_file.read_text(encoding="utf-8"))
    root = config_file.resolve().parents[1] if config_file.parent.name == "configs" else config_file.resolve().parent
    output = Path(data.get("output_root", "artifacts/datasets"))
    if not output.is_absolute():
        output = root / output
    output.mkdir(parents=True, exist_ok=True)
    limit = int(data.get("max_records_per_source", 1000))
    selected = selected or set(DATASETS)
    unknown = selected - DATASETS.keys()
    if unknown:
        raise ValueError("Unknown data source: " + ", ".join(sorted(unknown)))
    summary = {"schema": "spatialmind-data-pipeline-v1",
               "coverage": "local files only, not all sources necessarily downloaded",
               "sources": {}}
    for source in sorted(selected):
        spec = data.get("sources", {}).get(source, {})
        path = Path(spec.get("path", "data/" + source))
        if not path.is_absolute():
            path = root / path
        if not path.is_dir():
            summary["sources"][source] = {
                "status": "missing_local_data",
                "expected_path": str(path),
                "official_url": DATASETS[source].upstream,
                "requires_approval": DATASETS[source].needs_approval,
            }
            continue
        try:
            prepared = prepare_dataset(source, path, output / source, limit=limit)
            status = {"status": "indexed", "manifest": str(output / source / "manifest.json"),
                      "record_count": prepared["record_count"],
                      "oracle_count": prepared["oracle_count"]}
            calibration = spec.get("calibration")
            if with_replay and source in ("tum", "openloris"):
                if not calibration:
                    status["replay"] = "skipped_no_calibration"
                else:
                    camera = Path(calibration)
                    if not camera.is_absolute():
                        camera = root / camera
                    if camera.exists():
                        replay = replay_rgbd(
                            manifest=output/source/"manifest.json",
                            camera_file=camera, output=output/source/"replay",
                            limit=min(limit, 300))
                        status["replay"] = replay
                    else:
                        status["replay"] = "skipped_calibration_missing"
            summary["sources"][source] = status
        except Exception as exc:
            summary["sources"][source] = {
                "status": "failed", "error": str(exc),
                "expected_path": str(path),
            }
    (output / "pipeline_status.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    if fail_on_missing and any(
        item["status"] != "indexed" for item in summary["sources"].values()
    ):
        raise SystemExit("One or more requested datasets not present or invalid")
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="configs/datasets.example.json")
    parser.add_argument("--dataset", action="append", choices=sorted(DATASETS))
    parser.add_argument("--with-replay", action="store_true")
    parser.add_argument("--fail-on-missing", action="store_true")
    args = parser.parse_args()
    report = run(Path(args.config), selected=set(args.dataset) if args.dataset else None,
                 with_replay=args.with_replay, fail_on_missing=args.fail_on_missing)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
