"""Model call governance, event-triggered inference and reproducible cost telemetry.

Models never run in the robot control loop; they are invoked by semantic
events only. Cache keys include task/context/map revisions to prevent stale
decisions, and safety decisions cannot be served from an LLM cache.
"""
from __future__ import annotations

import hashlib
import json
import time
from collections import OrderedDict
from dataclasses import dataclass
from typing import Any, Callable


MODEL_EVENTS=frozenset({
    "task_created","user_revision","memory_conflict",
    "semantic_search_exhausted","task_level_failure",
})


@dataclass(frozen=True)
class InferenceUsage:
    model_id:str
    trigger:str
    latency_ms:float
    prompt_tokens:int
    completion_tokens:int
    estimated_usd:float
    cache_hit:bool=False


class InferenceGovernor:
    def __init__(
        self, *,
        max_calls:int=40, max_total_tokens:int=12000,
        max_cost_usd:float=1.0, cache_capacity:int=64,
        ttl_seconds:float=180.0,
        prompt_usd_per_million:float=0.0,
        completion_usd_per_million:float=0.0,
        clock:Callable[[],float]=time.monotonic,
    ):
        if min(max_calls,max_total_tokens,cache_capacity)<=0 or max_cost_usd<=0 or ttl_seconds<=0:
            raise ValueError("Invalid model budget")
        self.max_calls,self.max_total_tokens,self.max_cost_usd=(
            max_calls,max_total_tokens,max_cost_usd)
        self.cache_capacity,self.ttl_seconds=cache_capacity,ttl_seconds
        self.prompt_price=prompt_usd_per_million
        self.completion_price=completion_usd_per_million
        self.clock=clock
        self.calls=0
        self.total_tokens=0
        self.estimated_cost_usd=0.0
        self.history:list[InferenceUsage]=[]
        self._cache:OrderedDict[str,tuple[float,dict[str,Any]]]=OrderedDict()

    def permit(self,trigger:str)->bool:
        return trigger in MODEL_EVENTS and self.calls<self.max_calls and (
            self.total_tokens<self.max_total_tokens and
            self.estimated_cost_usd<self.max_cost_usd)

    @staticmethod
    def key(*,task_id:str,goal:str,map_version:str,
            memory_revision:str,model_id:str) -> str:
        raw=json.dumps({
            "task_id":task_id,"goal":goal,"map_version":map_version,
            "memory_revision":memory_revision,"model_id":model_id,
        },sort_keys=True)
        return hashlib.sha256(raw.encode()).hexdigest()

    def read(self,key:str,trigger:str) -> dict[str,Any] | None:
        if trigger not in MODEL_EVENTS or trigger=="memory_conflict":
            return None
        item=self._cache.get(key)
        if item is None:
            return None
        at,result=item
        if self.clock()-at>self.ttl_seconds:
            self._cache.pop(key,None)
            return None
        self._cache.move_to_end(key)
        self.history.append(InferenceUsage("cache",trigger,0,0,0,0,True))
        return dict(result)

    def record(
        self,*,model_id:str,trigger:str,latency_ms:float,
        prompt_tokens:int,completion_tokens:int,
        result:dict[str,Any],cache_key:str|None=None,
    )->InferenceUsage:
        if trigger not in MODEL_EVENTS:
            raise ValueError("Model invoked outside approved semantic events")
        if min(latency_ms,prompt_tokens,completion_tokens)<0:
            raise ValueError("Negative usage not allowed")
        new_tokens=prompt_tokens+completion_tokens
        estimated=(prompt_tokens*self.prompt_price+
                   completion_tokens*self.completion_price)/1_000_000
        if (self.calls>=self.max_calls
                or self.total_tokens+new_tokens>self.max_total_tokens
                or self.estimated_cost_usd+estimated>self.max_cost_usd):
            raise RuntimeError("Model inference budget exceeded")
        self.calls+=1
        self.total_tokens+=new_tokens
        self.estimated_cost_usd+=estimated
        usage=InferenceUsage(model_id,trigger,latency_ms,
                             prompt_tokens,completion_tokens,estimated)
        self.history.append(usage)
        if cache_key and trigger not in ("memory_conflict","user_revision"):
            self._cache[cache_key]=(self.clock(),dict(result))
            self._cache.move_to_end(cache_key)
            while len(self._cache)>self.cache_capacity:
                self._cache.popitem(last=False)
        return usage

    def metrics(self)->dict[str,Any]:
        latencies=sorted(x.latency_ms for x in self.history if not x.cache_hit)
        return {
            "calls":self.calls,"cache_hits":sum(x.cache_hit for x in self.history),
            "tokens":self.total_tokens,
            "estimated_usd":round(self.estimated_cost_usd,7),
            "p50_model_latency_ms":latencies[(len(latencies)-1)//2] if latencies else None,
            "p95_model_latency_ms":latencies[
                max(0,int(.95*len(latencies)+.999999)-1)] if latencies else None,
        }
