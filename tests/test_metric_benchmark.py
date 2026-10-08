"""Regression tests for paired metric interventions and TF optical projection."""
from __future__ import annotations

import math
import json
import tempfile
import unittest
from pathlib import Path
from spatialmind.vision import (
    CameraModel, BoundingBox, project_optical_bbox,
    rotate_xyz, visible_optical_depth_cells,
)
from spatialmind.physical import RoomMap
from spatialmind.physical_benchmark import run_physical_benchmark


class OpticalProjectionTests(unittest.TestCase):
    def test_quaternion_rotates_into_metric_map(self):
        # 90 degrees about Y: optical Z points to map X.
        q=(0,math.sin(math.pi/4),0,math.cos(math.pi/4))
        transformed=rotate_xyz((0,0,2),q)
        self.assertAlmostEqual(transformed[0],2)
        self.assertAlmostEqual(transformed[2],0,places=5)
        camera=CameraModel(8,8,4,4,3.5,3.5)
        result=project_optical_bbox(
            BoundingBox(2,2,6,6,"blue toolbox"),[2.0]*64,camera,
            (1,2,0),q,stamp=100.0)
        self.assertAlmostEqual(result.pose.x,3)
        self.assertAlmostEqual(result.pose.y,2)
        self.assertEqual(result.pose.frame_id,"map")
        cells=visible_optical_depth_cells(
            [2.0]*64,camera,(1,2,0),q,stride=2)
        self.assertTrue(cells)

    def test_bad_quaternion_rejected(self):
        with self.assertRaises(ValueError):
            rotate_xyz((1,2,3),(0,0,0,0))

    def test_calibration_safety_gate(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/"map.json"
            path.write_text(json.dumps({
                "calibrated":False,"rooms":{"office":[{"x":1,"y":2}]} }))
            with self.assertRaises(ValueError):
                RoomMap.from_json(str(path))
            path.write_text(json.dumps({
                "calibrated":True,"rooms":{"office":[{"x":1,"y":2}]} }))
            self.assertEqual(RoomMap.from_json(str(path)).room_for(
                RoomMap.from_json(str(path)).search_waypoints()[0][1]),"office")


class MetricBenchmarkTests(unittest.TestCase):
    def test_multi_policy_and_evidence_artifacts(self):
        with tempfile.TemporaryDirectory() as path:
            report=run_physical_benchmark(seeds=1,output_dir=path,trace=True)
            self.assertEqual(report["trial_count"],28)
            self.assertEqual(len(report["by_scenario"]),7)
            self.assertEqual(len(report["aggregate"]),4)
            self.assertTrue((Path(path)/"metrics.json").exists())
            self.assertTrue((Path(path)/"comparison.svg").exists())
            self.assertEqual(len(list((Path(path)/"traces").glob("*.json"))),28)
            self.assertEqual(sum(x["n"] for x in report["aggregate"].values()),28)

if __name__=="__main__":
    unittest.main()
