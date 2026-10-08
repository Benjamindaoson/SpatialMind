"""SpatialMind licensed, provenance-aware dataset ingestion.

Importing this package performs no network access, downloads or heavy imports.
"""
from .registry import DATASETS, dataset_info
from .core import DatasetRecord, read_records, prepare_dataset
__all__ = ["DATASETS", "dataset_info", "DatasetRecord", "read_records", "prepare_dataset"]
