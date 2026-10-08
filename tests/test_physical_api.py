"""Physical async HTTP actor smoke test."""
from __future__ import annotations
import os
import tempfile
import time
import unittest
from fastapi.testclient import TestClient
from spatialmind.physical_api import app


class PhysicalAPITests(unittest.TestCase):
    def test_nonblocking_task_and_observation(self):
        with tempfile.TemporaryDirectory() as d:
            old=os.environ.get("SPATIALMIND_DATA_DIR")
            os.environ["SPATIALMIND_DATA_DIR"]=d
            try:
                with TestClient(app) as client:
                    self.assertIn("Physical Lab",client.get("/").text)
                    self.assertEqual(client.get("/healthz").status_code,200)
                    start=client.post("/api/physical/tasks",json={
                        "kind":"find","target":"blue toolbox"})
                    self.assertEqual(start.status_code,200)
                    data=None
                    for _ in range(30):
                        data=client.get("/api/physical/state").json()
                        if data["task_status"]!="running":
                            break
                        time.sleep(.02)
                    self.assertEqual(data["task_status"],"succeeded")
                    self.assertIsNotNone(data["result"]["evidence_ref"])
                    self.assertTrue(data["memory"])
            finally:
                if old is None:
                    os.environ.pop("SPATIALMIND_DATA_DIR",None)
                else:
                    os.environ["SPATIALMIND_DATA_DIR"]=old

if __name__=="__main__":
    unittest.main()
