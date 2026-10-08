"""Low-overhead process resource accounting and storage quota enforcement."""
from __future__ import annotations

import json
import os
import resource
import time
from contextlib import contextmanager
from pathlib import Path


class ResourceLedger:
    def __init__(self):
        self.spans:dict[str,list[float]]={}
        self.started=time.monotonic()

    @contextmanager
    def span(self,name:str):
        start=time.perf_counter()
        try:
            yield
        finally:
            duration=(time.perf_counter()-start)*1000
            self.spans.setdefault(name,[]).append(duration)

    def report(self)->dict:
        usage=resource.getrusage(resource.RUSAGE_SELF)
        raw_rss=usage.ru_maxrss
        # Linux reports kilobytes; macOS reports bytes.
        peak_mib=raw_rss/(1024*1024 if os.uname().sysname=="Darwin" else 1024)
        durations={}
        for kind,values in self.spans.items():
            sorted_values=sorted(values)
            index=max(0,int(.95*len(values)+.99999)-1)
            durations[kind]={
                "calls":len(values),"p50_ms":sorted_values[(len(values)-1)//2],
                "p95_ms":sorted_values[index],
                "total_ms":sum(values),
            }
        return {
            "elapsed_s":time.monotonic()-self.started,
            "peak_rss_mib":round(peak_mib,3),
            "cpu_user_s":usage.ru_utime,
            "cpu_system_s":usage.ru_stime,
            "spans":durations,
        }


class DiskQuota:
    """Bound log growth without deleting evidence automatically."""
    def __init__(self,root:Path,limit_gib:float=140):
        if limit_gib<=0:
            raise ValueError("Quota must be positive")
        self.root=Path(root).resolve()
        self.limit_bytes=int(limit_gib*1024**3)

    def used_bytes(self):
        return sum(path.stat().st_size for path in self.root.rglob("*")
                   if path.is_file() and not path.is_symlink()) if self.root.exists() else 0

    def ensure_capacity(self,additional_bytes:int=0):
        if additional_bytes<0:
            raise ValueError("Invalid size")
        if self.used_bytes()+additional_bytes>self.limit_bytes:
            raise RuntimeError("Dataset quota reached; archive or prune with human approval")

    def manifest(self):
        return {"root":str(self.root),"quota_bytes":self.limit_bytes,
                "used_bytes":self.used_bytes()}

    def write_manifest(self,path:Path):
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(self.manifest(),indent=2),encoding="utf-8")
