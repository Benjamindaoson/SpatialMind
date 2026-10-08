"""Scene assets are a consistent, syntax-valid world/map, not runtime Gazebo proof."""
from __future__ import annotations
import json
import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree
from spatialmind.physical import RoomMap


class SceneGenerationTest(unittest.TestCase):
    def test_geometry_and_occupancy_alignment(self):
        from scripts.generate_gazebo_scene import generate
        with tempfile.TemporaryDirectory() as temp:
            directory=Path(temp)
            generate(directory)
            root=ElementTree.parse(directory/"room_world.sdf.xacro").getroot()
            names=[m.get("name") for m in root.findall(".//model")]
            self.assertIn("blue_toolbox",names)
            self.assertIn("red_first_aid",names)
            head=(directory/"map.pgm").read_text(encoding="ascii").splitlines()
            self.assertEqual(head[0],"P2")
            self.assertEqual(head[2],"120 120")
            self.assertTrue((directory/"map.yaml").exists())
            parsed=RoomMap.from_json(str(directory/"semantic_map.json"))
            self.assertEqual(len(parsed.search_waypoints()),8)
            self.assertEqual(json.loads((directory/"manifest.json").read_text())["origin_m"],[-6,-6])

    def test_rgbd_sensor_injected_into_robot(self):
        from scripts.build_rgbd_robot import inject_rgbd
        root=ElementTree.fromstring(
            '<sdf version="1.9"><model name="bot"><link name="base_link"/></model></sdf>')
        inject_rgbd(root)
        sensor=root.find(".//sensor[@name='spatialmind_rgbd']")
        self.assertEqual(sensor.get("type"),"rgbd_camera")
        self.assertEqual(sensor.findtext("topic"),"/rgbd_camera")

if __name__=="__main__":
    unittest.main()
