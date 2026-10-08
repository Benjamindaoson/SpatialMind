#!/usr/bin/env python3
"""Attach an RGBD camera to the official Gazebo TB3 robot SDF.

Run only on an installed ROS2 Jazzy/Nav2 environment with xacro.
The generated sensor must still be checked in Gazebo for topic names and TF.
"""
from __future__ import annotations

import argparse
import subprocess
from pathlib import Path
from xml.etree import ElementTree as ET


def text_element(parent,tag,value):
    node=ET.SubElement(parent,tag)
    node.text=str(value)
    return node


def inject_rgbd(root:ET.Element) -> ET.Element:
    model=root.find(".//model")
    if model is None:
        raise ValueError("Expected Gazebo SDF model")
    link=model.find("./link[@name='base_link']")
    if link is None:
        link=model.find("./link")
    if link is None:
        raise ValueError("No SDF robot link available")
    if link.find("./sensor[@name='spatialmind_rgbd']") is not None:
        raise ValueError("RGB-D camera already attached")
    sensor=ET.SubElement(link,"sensor",{
        "name":"spatialmind_rgbd","type":"rgbd_camera",
    })
    text_element(sensor,"pose","0.12 0 0.24 0 0 0")
    text_element(sensor,"always_on","1")
    text_element(sensor,"update_rate","10")
    text_element(sensor,"topic","/rgbd_camera")
    text_element(sensor,"gz_frame_id","front_rgbd_optical_frame")
    camera=ET.SubElement(sensor,"camera")
    text_element(camera,"horizontal_fov","1.047")
    image=ET.SubElement(camera,"image")
    text_element(image,"width","320")
    text_element(image,"height","240")
    text_element(image,"format","R8G8B8")
    clip=ET.SubElement(camera,"clip")
    text_element(clip,"near",".12")
    text_element(clip,"far","6.0")
    text_element(camera,"optical_frame_id","front_rgbd_optical_frame")
    return root


def build(input_xacro:Path,output:Path):
    if not input_xacro.exists():
        raise FileNotFoundError(input_xacro)
    expanded=subprocess.run(
        ["xacro",str(input_xacro)],check=True,capture_output=True,text=True
    ).stdout
    root=ET.fromstring(expanded)
    inject_rgbd(root)
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text('<?xml version="1.0"?>\n'+ET.tostring(
        root,encoding="unicode"),encoding="utf-8")


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--xacro",required=True,type=Path)
    parser.add_argument("--output",default="simulation/generated/robot_rgbd.sdf",
                        type=Path)
    args=parser.parse_args()
    build(args.xacro,args.output)
    print(args.output)
