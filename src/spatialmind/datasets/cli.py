"""Dataset ingestion commands; no download happens implicitly."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .core import prepare_dataset, read_records, sha256_file
from .registry import DATASETS, dataset_info


def main(argv: list[str] | None = None):
    parser = argparse.ArgumentParser(prog="spatialmind data")
    sub = parser.add_subparsers(dest="action", required=True)
    sources = sub.add_parser("sources", help="List official data and licensing requirements")
    sources.add_argument("--dataset", choices=list(DATASETS))
    ingest = sub.add_parser("prepare", help="Index local files without copying licensed assets")
    ingest.add_argument("--dataset", choices=list(DATASETS), required=True)
    ingest.add_argument("--source", required=True)
    ingest.add_argument("--output", required=True)
    ingest.add_argument("--limit", type=int, default=1000)
    ingest.add_argument("--rgb-index")
    ingest.add_argument("--depth-index")
    ingest.add_argument("--pose-index")
    ingest.add_argument("--max-delta-s", type=float, default=.04)
    ingest.add_argument("--max-pose-delta-s", type=float, default=.05)
    verify = sub.add_parser("verify", help="Check index digest, count and local paths")
    verify.add_argument("--manifest", required=True)
    replay = sub.add_parser("replay", help="Offline real RGB-D pixel/depth replay")
    replay.add_argument("--manifest", required=True)
    replay.add_argument("--camera", required=True)
    replay.add_argument("--output", required=True)
    replay.add_argument("--limit", type=int, default=300)
    replay.add_argument("--min-pixels", type=int, default=8)
    args = parser.parse_args(argv)
    if args.action == "sources":
        values = [dataset_info(args.dataset)] if args.dataset else DATASETS.values()
        result = [v.to_dict() for v in values]
    elif args.action == "prepare":
        opts = {}
        if args.dataset in ("tum", "openloris"):
            opts = {
                "rgb_index": args.rgb_index, "depth_index": args.depth_index,
                "pose_index": args.pose_index, "max_delta_s": args.max_delta_s,
                "max_pose_delta_s": args.max_pose_delta_s,
            }
        result = prepare_dataset(args.dataset, args.source, args.output,
                                 limit=args.limit, **opts)
    elif args.action == "verify":
        doc = json.loads(Path(args.manifest).read_text(encoding="utf-8"))
        path = Path(doc["records_path"])
        verified = sha256_file(path) == doc["records_sha256"]
        count = sum(1 for _ in read_records(path))
        verified = verified and count == doc["record_count"]
        if doc.get("oracle_sha256"):
            verified = verified and sha256_file(
                Path(doc["oracle_path"])) == doc["oracle_sha256"]
        result = {"verified": bool(verified), "count": count, "dataset": doc["dataset"],
                  "oracle_count": doc.get("oracle_count", 0)}
        if not verified:
            raise SystemExit("FAIL: records/oracle SHA256 or record count mismatch")
    else:
        from .replay import replay_rgbd
        result = replay_rgbd(manifest=args.manifest, camera_file=args.camera,
                             output=args.output, limit=args.limit,
                             min_pixels=args.min_pixels)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return result
