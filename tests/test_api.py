"""Local API smoke tests. Install extras with: pip install -e '.[api,dev]'."""
from __future__ import annotations
import unittest
try:
    from fastapi.testclient import TestClient
    from spatialmind.api import app
except ImportError:
    TestClient = None
    app = None

@unittest.skipUnless(TestClient is not None, "API and httpx extras not installed")
class APIContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.client = TestClient(app)

    @classmethod
    def tearDownClass(cls):
        cls.client.close()

    def test_dashboard_renders(self):
        response = self.client.get("/")
        self.assertEqual(response.status_code, 200)
        self.assertIn("SpatialMind", response.text)

    def test_health_and_state(self):
        self.assertEqual(self.client.get("/healthz").json()["status"], "ok")
        state = self.client.get("/api/state").json()
        self.assertEqual(state["backend"], "grid-simulation")
        self.assertEqual(len(state["grid"]), 12)

    def test_clarify_then_execute(self):
        first = self.client.post("/api/chat", json={"message": "Find a box"})
        self.assertEqual(first.status_code, 200)
        self.assertEqual(first.json()["kind"], "clarification")
        session_id = first.json()["session_id"]
        second = self.client.post("/api/chat", json={
            "message": "Find the blue toolbox", "session_id": session_id
        })
        self.assertEqual(second.status_code, 200)
        data = second.json()
        self.assertEqual(data["kind"], "result")
        self.assertEqual(data["session_id"], session_id)
        self.assertEqual(data["result"]["status"], "succeeded")
        self.assertIsNotNone(data["result"]["evidence_id"])

    def test_invalid_room_returns_error(self):
        response = self.client.post("/api/world/move/toolbox_1/mars")
        self.assertEqual(response.status_code, 422)

if __name__ == "__main__":
    unittest.main()
