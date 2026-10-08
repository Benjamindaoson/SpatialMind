"""Normalized dataset record/index format with bounded, auditable file references."""
from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Iterator

from .registry import dataset_info

SCHEMA_VERSION = "spatialmind-dataset-v1"


def safe_input_file(root: Path, filename: str) -> Path:
    """Reject absolute paths, parent traversals and symlink exits."""
    if not isinstance(filename, str) or not filename or "\\x00" in filename:
        raise ValueError("Invalid file reference")
    requested = Path(filename)
    if requested.is_absolute() or ".." in requested.parts:
        raise ValueError("Absolute paths and parent traversal are not permitted")
    root_abs = root.resolve()
    target = (root_abs / requested).resolve()
    if not target.is_relative_to(root_abs):
        raise ValueError("Data path escapes registered input directory")
    if not target.is_file():
        raise FileNotFoundError(target)
    return target


@dataclass(frozen=True)
class DatasetRecord:
    dataset: str
    kind: str
    id: str
    sequence: str
    split: str = "unspecified"
    timestamp: float | None = None
    data: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        dataset_info(self.dataset)
        if not self.kind or not self.id or not self.sequence:
            raise ValueError("Record needs kind, id and sequence")

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def read_records(path: Path | str) -> Iterator[DatasetRecord]:
    with Path(path).open(encoding="utf-8") as stream:
        for n, line in enumerate(stream, 1):
            if not line.strip():
                continue
            obj = json.loads(line)
            yield DatasetRecord(**obj)


def sha256_file(path: Path, *, chunk_size: int = 1 << 20) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(chunk_size), b""):
            digest.update(block)
    return digest.hexdigest()


def prepare_dataset(
    name: str, source: Path | str, output: Path | str, *,
    limit: int = 1000, **options: Any,
) -> dict[str, Any]:
    """Convert source metadata/indexes without copying raw licensed files.

    If no valid data exists, fail explicitly rather than emitting synthetic
    dataset records as if they had been downloaded.
    """
    if limit < 1:
        raise ValueError("limit must be positive")
    info = dataset_info(name)
    source = Path(source).resolve()
    if not source.is_dir():
        raise FileNotFoundError(f"Unpacked dataset directory missing: {source}")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    if name in {"tum", "openloris"}:
        from .rgbd import rgbd_records
        records = rgbd_records(name, source, limit=limit, **options)
    elif name == "3rscan":
        from .scans import scan_records
        records = scan_records(source, limit=limit)
    elif name in {"replicacad", "hm3d", "goat"}:
        from .habitat import habitat_records
        records = habitat_records(name, source, limit=limit)
    else:
        raise ValueError(name)
    records_file = output / "records.jsonl"
    oracle_file = output / "oracle.jsonl"
    total, oracle_count, counts = 0, 0, {}
    with records_file.open("w", encoding="utf-8") as policy, oracle_file.open(
        "w", encoding="utf-8"
    ) as evaluator:
        for record in records:
            if record.dataset != name:
                raise ValueError("Adapter dataset mismatch")
            serialized = json.dumps(record.to_dict(), ensure_ascii=False, sort_keys=True) + "\\n"
            if record.kind.startswith("oracle_"):
                evaluator.write(serialized)
                oracle_count += 1
            else:
                if total >= limit:
                    break
                policy.write(serialized)
                total += 1
                counts[record.kind] = counts.get(record.kind, 0) + 1
    if not total:
        records_file.unlink(missing_ok=True)
        oracle_file.unlink(missing_ok=True)
        raise ValueError(f"No usable {name} data in {source}. Check expected file layout.")
    if not oracle_count:
        oracle_file.unlink(missing_ok=True)
    manifest = {
        "schema": SCHEMA_VERSION,
        "dataset": name,
        "upstream": info.upstream,
        "rights": info.rights,
        "source_root": str(source),
        "records_path": str(records_file.resolve()),
        "record_count": total,
        "oracle_count": oracle_count,
        "oracle_path": str(oracle_file.resolve()) if oracle_count else None,
        "oracle_sha256": sha256_file(oracle_file) if oracle_count else None,
        "record_kinds": counts,
        "records_sha256": sha256_file(records_file),
        "limit": limit,
        "status": "indexed_local_files_not_navigation_evaluated",
        "notice": "Indexes only: no licensed raw data copied, no simulator success inferred.",
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest
