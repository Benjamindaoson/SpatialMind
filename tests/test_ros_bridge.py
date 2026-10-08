"""ROS2 RGB-D bridge decoding with fake message structs, no ROS install."""
from __future__ import annotations
import importlib.util
import math
import struct
import unittest
from pathlib import Path
from types import SimpleNamespace

MODULE=Path(__file__).resolve().parents[1]/"ros2_ws/src/spatialmind_ros/spatialmind_ros/perception_bridge.py"
spec=importlib.util.spec_from_file_location("perception_bridge_under_test",MODULE)
bridge=importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


class ImageCodecTests(unittest.TestCase):
    def test_pixel_and_depth_decoding(self):
        stamp=SimpleNamespace(sec=4,nanosec=500)
        header=SimpleNamespace(frame_id="front_rgbd_optical_frame",stamp=stamp)
        rgb=bytearray([15,15,15]*(12*12))
        for y in range(4,8):
            for x in range(3,7):
                i=3*(y*12+x)
                rgb[i:i+3]=bytes((5,8,230))
        color=SimpleNamespace(
            encoding="rgb8",width=12,height=12,step=36,
            data=bytes(rgb),header=header)
        raw_depth=struct.pack("<"+"f"*144,*([2.]*144))
        depth=SimpleNamespace(
            encoding="32FC1",width=12,height=12,step=48,
            data=raw_depth,is_bigendian=0)
        camera=SimpleNamespace(
            width=12,height=12,
            k=[6.,0,5.5,0,6.,5.5,0,0,1])
        self.assertEqual(len(bridge.rgb8_bytes(color)),432)
        self.assertEqual(len(bridge.depth_float_m(depth)),144)
        q=(0,math.sin(math.pi/4),0,math.cos(math.pi/4))
        result=bridge.build_payload(
            color,depth,camera,(1,1,0),q,frame_id="map")
        self.assertEqual(len(result["detections"]),1)
        self.assertEqual(result["detections"][0]["label"],"blue toolbox")
        self.assertEqual(result["frame_id"],"map")
        self.assertTrue(result["visible_cells"])
        self.assertGreater(result["coverage_quality"],.8)
        self.assertIn("rgbd:4.500",result["detections"][0]["evidence_ref"])

    def test_reject_misaligned_depth_camera(self):
        frame=SimpleNamespace(
            encoding="rgb8",width=4,height=4,step=12,
            data=bytes([0]*48),
            header=SimpleNamespace(frame_id="camera",
                stamp=SimpleNamespace(sec=1,nanosec=0)))
        depth=SimpleNamespace(
            width=8,height=4,
            encoding="16UC1",step=16,is_bigendian=0,data=bytes(64))
        info=SimpleNamespace(width=4,height=4,k=[1.,0.,0.,0.,1.,0.,0.,0.,1.])
        with self.assertRaises(ValueError):
            bridge.build_payload(frame,depth,info,(0,0,0),(0,0,0,1))

if __name__=="__main__":
    unittest.main()
