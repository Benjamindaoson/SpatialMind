"""Deterministic contracts. No model keys, GPU, ROS 2 or network needed."""
from __future__ import annotations
import json
import tempfile
import unittest
from pathlib import Path
from spatialmind.benchmark import run_suite, run_trial
from spatialmind.context import ContextEngine
from spatialmind.language import ClarificationNeeded, RuleBasedInterpreter
from spatialmind.memory import SpatialMemory
from spatialmind.models import Detection, Observation, Point, TaskRequest, TaskStatus
from spatialmind.runtime import AgentRuntime
from spatialmind.telemetry import EventStore
from spatialmind.world import GridWorld, SimRobot


class GridTests(unittest.TestCase):
    def test_room_navigation(self):
        world = GridWorld()
        robot = SimRobot(world)
        result = robot.navigate(world.room_center("meeting"))
        self.assertTrue(result.success)
        self.assertGreater(result.steps, 0)
        self.assertEqual(world.room_at(robot.pose), "meeting")

    def test_blockage_and_recovery(self):
        world = GridWorld()
        world.set_obstacle(Point(9, 3))
        robot = SimRobot(world)
        dest = world.room_center("meeting")
        first = robot.navigate(dest)
        self.assertFalse(first.success)
        self.assertEqual(first.reason, "blocked")
        self.assertTrue(robot.navigate(dest).success)

    def test_wall_occludes(self):
        world = GridWorld()
        world.move_object("toolbox_1", Point(11, 2))
        robot = SimRobot(world, Point(8, 2), sensor_range=10)
        self.assertFalse(any(d.label == "blue toolbox" for d in robot.observe().detections))

    def test_invalid_move(self):
        with self.assertRaises(ValueError):
            GridWorld().move_object("toolbox_1", Point(0, 0))


class MemoryTests(unittest.TestCase):
    def test_update_and_missing(self):
        with SpatialMemory() as memory:
            memory.update(Observation(
                Point(2, 2), "lab",
                (Detection("blue toolbox", Point(3, 3), track_id="T1"),),
                frozenset({Point(3, 3)}),
            ))
            memory.update(Observation(Point(2, 2), "lab", (), frozenset({Point(1, 1)})))
            self.assertEqual(memory.get("T1").status, "observed")
            memory.update(Observation(Point(2, 2), "lab", (), frozenset({Point(3, 3)})))
            self.assertEqual(memory.get("T1").status, "missing")
            memory.update(Observation(
                Point(16, 9), "office",
                (Detection("blue toolbox", Point(16, 9), track_id="T1", room="office"),),
                frozenset({Point(16, 9)}),
            ))
            self.assertEqual(memory.get("T1").position, Point(16, 9))
            self.assertEqual(memory.get("T1").status, "observed")
            self.assertEqual(memory.get("T1").room, "office")
            self.assertEqual(memory.get("T1").sightings, 2)

    def test_obstacle_does_not_erase_occluded_object(self):
        world = GridWorld()
        world.move_object("toolbox_1", Point(4, 2))
        robot = SimRobot(world, sensor_range=4)
        with SpatialMemory() as memory:
            memory.update(robot.observe())
            self.assertIsNotNone(memory.get("toolbox_1"))
            world.set_obstacle(Point(3, 2))
            obs = robot.observe()
            self.assertNotIn(Point(4, 2), obs.visible_cells)
            memory.update(obs)
            self.assertEqual(memory.get("toolbox_1").status, "observed")


class AgentTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.world = GridWorld()
        self.robot = SimRobot(self.world)
        self.memory = SpatialMemory(Path(self.dir.name) / "memory.db")
        self.events = EventStore(Path(self.dir.name) / "events.db")
        self.agent = AgentRuntime(self.robot, self.world, self.memory, self.events)

    def tearDown(self):
        self.memory.close()
        self.events.close()
        self.dir.cleanup()

    def test_initial_observation_is_evidence(self):
        result = self.agent.run("Find the blue toolbox")
        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertIsNotNone(result.evidence_id)
        self.assertTrue(self.memory.observation(result.evidence_id)["detections"])
        self.assertTrue(self.events.events(result.task_id))

    def test_navigate_to_room(self):
        result = self.agent.run("Go to the meeting room")
        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertEqual(self.world.room_at(self.robot.pose), "meeting")

    def test_checkpoint_resume(self):
        self.world.move_object("toolbox_1", Point(16, 9))
        interrupted = self.agent.run("Find the blue toolbox", max_actions=1)
        self.assertEqual(interrupted.status, TaskStatus.INTERRUPTED)
        completed = self.agent.run(task_id=interrupted.task_id, resume=True, max_actions=120)
        self.assertEqual(completed.task_id, interrupted.task_id)
        self.assertEqual(completed.status, TaskStatus.SUCCEEDED)

    def test_detects_moved_object(self):
        self.memory.update(self.robot.observe())
        self.world.move_object("toolbox_1", Point(16, 9))
        result = self.agent.run("Find the blue toolbox", max_actions=120)
        self.assertEqual(result.status, TaskStatus.SUCCEEDED)
        self.assertEqual(self.memory.get("toolbox_1").position, Point(16, 9))

    def test_duplicate_task_id_refused(self):
        self.agent.run("Find the blue toolbox", task_id="same")
        with self.assertRaises(ValueError):
            self.agent.run("Find the blue toolbox", task_id="same")

    def test_context_budget_preserves_invariants(self):
        self.memory.update(self.robot.observe())
        state = ContextEngine(self.memory, 320).build(
            TaskRequest("find", "blue toolbox"), (2, 2),
            [{"type":"log", "data":{"details":"x"*800}}]*10,
        )
        self.assertLessEqual(len(json.dumps(state)), 320)
        self.assertEqual(state["goal"]["target"], "blue toolbox")

    def test_clarification(self):
        with self.assertRaises(ClarificationNeeded):
            RuleBasedInterpreter().parse("Find a box")


class BenchmarkTests(unittest.TestCase):
    def test_determinism(self):
        self.assertEqual(run_trial("moved_toolbox"), run_trial("moved_toolbox"))

    def test_policy_comparison_report(self):
        report = run_suite(scenarios=["find_toolbox", "find_first_aid"])
        self.assertEqual(len(report["trials"]), 6)
        self.assertEqual(set(report["metrics"]), {"full", "no_memory", "uniform_search"})
        self.assertIn("toy-grid", report["notice"])

if __name__ == "__main__":
    unittest.main()
