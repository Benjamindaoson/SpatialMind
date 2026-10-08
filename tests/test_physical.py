"""Frame, RGB-D, belief updates and async robot execution: no ROS/GPU required."""
from __future__ import annotations

import asyncio
import math
import tempfile
import unittest
from pathlib import Path

from spatialmind.async_agent import AsyncPhysicalAgent, Mission
from spatialmind.belief import BeliefMemory
from spatialmind.metric_sim import MetricSimRobot, demo_room_map
from spatialmind.physical import (
    FrameTransform, MetricObservation, ObjectEstimate, Pose2D, RoomMap,
)
from spatialmind.telemetry import EventStore
from spatialmind.vision import BoundingBox, CameraModel, RGBDProjector, detect_color_regions


class GeometryTests(unittest.TestCase):
    def test_frame_transform_and_frame_guard(self):
        transformed=FrameTransform("odom","map",4,5,math.pi/2).apply(
            Pose2D(1,2,0,"odom"))
        self.assertAlmostEqual(transformed.x,2)
        self.assertAlmostEqual(transformed.y,6)
        with self.assertRaises(ValueError):
            transformed.distance(Pose2D(0,0,frame_id="odom"))

    def test_camera_depth_project_not_ground_truth(self):
        camera=CameraModel(8,8,4,4,3.5,3.5)
        estimate=RGBDProjector(camera).project(
            BoundingBox(2,2,6,6,"blue toolbox",0.9),[2.0]*64,Pose2D(1,1))
        self.assertIsNotNone(estimate)
        self.assertAlmostEqual(estimate.pose.x,3)
        self.assertAlmostEqual(estimate.pose.y,1)
        self.assertGreater(estimate.pose.position_variance,0)

    def test_depth_nans_abstain(self):
        estimate=RGBDProjector(CameraModel(4,4,2,2,1.5,1.5)).project(
            BoundingBox(0,0,4,4,"target"),[float("nan")]*16,Pose2D(0,0))
        self.assertIsNone(estimate)

    def test_measured_color_detector(self):
        rgb=bytearray([30,30,30]*(10*10))
        for y in range(2,6):
            for x in range(3,7):
                i=3*(y*10+x)
                rgb[i:i+3]=bytes([10,10,255])
        boxes=detect_color_regions(bytes(rgb),10,10,min_pixels=8)
        self.assertEqual(len(boxes),1)
        self.assertEqual(boxes[0].label,"blue toolbox")


class BeliefTests(unittest.TestCase):
    def test_two_negative_views_before_missing(self):
        with tempfile.TemporaryDirectory() as tmp:
            memory=BeliefMemory(str(Path(tmp)/"belief.sqlite"))
            t=15000.0
            robot=Pose2D(0,0,stamp=t)
            target=ObjectEstimate("box",Pose2D(1,0,stamp=t),.95,
                                  track_id="box1",evidence_ref="rgb:1")
            memory.update(MetricObservation(robot,(target,),frozenset({(4,0)}),.95))
            self.assertEqual(memory.get("box1").status,"observed")
            no_coverage=MetricObservation(Pose2D(0,0,stamp=t+1),(),
                                          frozenset({(4,0)}),.99)
            memory.update(no_coverage)
            self.assertEqual(memory.get("box1").status,"observed")
            weak=MetricObservation(Pose2D(0,0,stamp=t+2),(),
                                   frozenset({(4,0)}),.1)
            memory.update(weak)
            self.assertEqual(memory.get("box1").status,"observed")
            seen=MetricObservation(Pose2D(0,0,stamp=t+3),(),
                                   frozenset({(4,0)}),.99)
            # belief grid resolution .25 -> x=1 becomes cell=(4,0)
            memory.update(seen)
            self.assertEqual(memory.get("box1").status,"uncertain")
            memory.update(MetricObservation(Pose2D(0,0,stamp=t+4),(),
                                            frozenset({(4,0)}),.99))
            self.assertEqual(memory.get("box1").status,"missing")
            memory.close()

    def test_multiple_objects_not_silently_merged(self):
        memory=BeliefMemory()
        t=20000.0
        obs=MetricObservation(Pose2D(0,0,stamp=t),(
            ObjectEstimate("tool",Pose2D(1,0,stamp=t),.95,track_id="a"),
            ObjectEstimate("tool",Pose2D(4,0,stamp=t),.95,track_id="b"),
        ))
        memory.update(obs)
        self.assertEqual(memory.count(),2)
        self.assertEqual(memory.get("a").pose.x,1)
        memory.close()


class AsyncAgentTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.robot=MetricSimRobot()
        self.memory=BeliefMemory(str(Path(self.tmp.name)/"memory.sqlite"))
        self.events=EventStore(str(Path(self.tmp.name)/"events.sqlite"))
        self.agent=AsyncPhysicalAgent(self.robot,demo_room_map(),self.memory,self.events)

    async def asyncTearDown(self):
        self.memory.close()
        self.events.close()
        self.tmp.cleanup()

    async def test_navigation_and_grounded_search(self):
        a=await self.agent.run(Mission("navigate","meeting",room="meeting"))
        self.assertEqual(a.status,"succeeded")
        b=await self.agent.run(Mission("find","red first aid kit"))
        self.assertEqual(b.status,"succeeded")
        self.assertIsNotNone(b.evidence_ref)
        self.assertTrue(any(x["type"]=="goal_verified" for x in self.events.events(b.task_id)))

    async def test_hidden_moved_target(self):
        self.robot.place("toolbox_a",Pose2D(17,9))
        outcome=await self.agent.run(Mission("find","blue toolbox",max_actions=20))
        self.assertEqual(outcome.status,"succeeded")
        self.assertGreater(outcome.motion_m,0)

    async def test_live_cancellation(self):
        self.robot.latency_s=.05
        task_id=await self.agent.start(Mission("navigate","meeting","meeting"))
        await asyncio.sleep(.015)
        await self.agent.cancel()
        result=await self.agent.join()
        self.assertEqual(result.task_id,task_id)
        self.assertEqual(result.status,"cancelled")

    async def test_mid_mission_revision(self):
        self.robot.latency_s=.04
        await self.agent.start(Mission("navigate","office","office"))
        await asyncio.sleep(.015)
        await self.agent.revise(Mission("navigate","lab","lab"))
        result=await self.agent.join()
        self.assertEqual(result.status,"succeeded")
        self.assertGreaterEqual(result.replans,1)
        self.assertTrue(any(x["type"]=="mission_revised" for x in self.events.events(result.task_id)))

if __name__ == "__main__":
    unittest.main()
