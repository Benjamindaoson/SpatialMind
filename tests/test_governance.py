from __future__ import annotations
import tempfile
import unittest
from pathlib import Path
from spatialmind.governance import InferenceGovernor
from spatialmind.resources import ResourceLedger, DiskQuota


class GovernanceTests(unittest.TestCase):
    def test_stale_context_invalidates_cache(self):
        now=[0.]
        gate=InferenceGovernor(clock=lambda:now[0],ttl_seconds=10)
        a=gate.key(task_id="1",goal="find box",map_version="1",
                   memory_revision="1",model_id="local")
        b=gate.key(task_id="1",goal="find box",map_version="1",
                   memory_revision="2",model_id="local")
        self.assertNotEqual(a,b)
        self.assertFalse(gate.permit("navigation_feedback"))
        self.assertTrue(gate.permit("task_created"))
        gate.record(model_id="local",trigger="task_created",latency_ms=100,
                    prompt_tokens=50,completion_tokens=20,
                    result={"kind":"find"},cache_key=a)
        self.assertIsNotNone(gate.read(a,"task_created"))
        self.assertIsNone(gate.read(b,"task_created"))
        self.assertIsNone(gate.read(a,"memory_conflict"))
        now[0]=11
        self.assertIsNone(gate.read(a,"task_created"))

    def test_budget_enforced_without_silent_overrun(self):
        gate=InferenceGovernor(max_calls=1,max_total_tokens=10,max_cost_usd=.01,
                               prompt_usd_per_million=1000)
        with self.assertRaises(RuntimeError):
            gate.record(model_id="remote",trigger="task_created",latency_ms=200,
                        prompt_tokens=12,completion_tokens=2,result={})
        self.assertEqual(gate.calls,0)
        gate.record(model_id="remote",trigger="task_created",latency_ms=100,
                    prompt_tokens=5,completion_tokens=2,result={})
        self.assertEqual(gate.metrics()["tokens"],7)
        self.assertFalse(gate.permit("task_created"))

    def test_resource_and_storage_budget(self):
        ledger=ResourceLedger()
        with ledger.span("vision"):
            sum(range(100))
        report=ledger.report()
        self.assertEqual(report["spans"]["vision"]["calls"],1)
        self.assertGreater(report["peak_rss_mib"],0)
        with tempfile.TemporaryDirectory() as d:
            quota=DiskQuota(Path(d),limit_gib=0.000001)
            (Path(d)/"events.json").write_text("x"*2000)
            with self.assertRaises(RuntimeError):
                quota.ensure_capacity()
