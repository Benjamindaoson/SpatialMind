#!/usr/bin/env python3
"""Generate a metric Gazebo Harmonic world, aligned occupancy map and waypoints.

Geometry and map are generated from one source of truth. Objects are *not*
painted into the occupancy map because they are potentially movable.
This creates assets but does not itself run Gazebo, Nav2 or RGB-D perception.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from xml.etree.ElementTree import Element,SubElement,tostring


ROOMS={
    "lab":[(-3.5,-3.5),(-3.6,-1.3)],
    "storage":[(-3.4,3.4),(-3.8,1.8)],
    "meeting":[(3.4,-3.2),(4.0,-1.7)],
    "office":[(3.6,3.3),(2.0,4.0)],
}
# Center, dimensions. External walls + internal dividers with meter-wide doors.
WALLS=[
    (0,-5.95,12,.1), (0,5.95,12,.1),
    (-5.95,0,.1,12), (5.95,0,.1,12),
    (0,-4.15,.12,3.6),(0,2.3,.12,7.2),
    (-3.87,0,4.0,.12),(2.7,0,6.4,.12),
]


def xml_text(parent,tag,value,**attrs):
    return SubElement(parent,tag,attrs,text=str(value)) if False else _elem(parent,tag,str(value),attrs)


def _elem(parent,tag,text,attrs):
    item=SubElement(parent,tag,attrs)
    item.text=text
    return item


def box(world,name,position,dimensions,color,*,static=True):
    model=SubElement(world,"model",name=name)
    xml_text(model,"pose"," ".join(str(x) for x in (*position,0,0,0)))
    xml_text(model,"static","true" if static else "false")
    link=SubElement(model,"link",name="body")
    for kind in ("collision","visual"):
        element=SubElement(link,kind,name=kind)
        geom=SubElement(element,"geometry")
        box=SubElement(geom,"box")
        xml_text(box,"size"," ".join(str(x) for x in dimensions))
        if kind=="visual":
            material=SubElement(element,"material")
            xml_text(material,"ambient",color)
            xml_text(material,"diffuse",color)
    return model


def make_world(path):
    sdf=Element("sdf",version="1.9",attrib={"xmlns:xacro":"http://www.ros.org/wiki/xacro"})
    world=SubElement(sdf,"world",name="spatialmind")
    for name in ("physics","user-commands","scene-broadcaster"):
        mapping={
            "physics":("gz-sim-physics-system","gz::sim::systems::Physics"),
            "user-commands":("gz-sim-user-commands-system","gz::sim::systems::UserCommands"),
            "scene-broadcaster":("gz-sim-scene-broadcaster-system","gz::sim::systems::SceneBroadcaster"),
        }
        SubElement(world,"plugin",filename=mapping[name][0],name=mapping[name][1])
    sensor_plugin=SubElement(world,"plugin",filename="gz-sim-sensors-system",
                            name="gz::sim::systems::Sensors")
    xml_text(sensor_plugin,"render_engine","ogre2")
    light=SubElement(world,"light",name="sun",type="directional")
    xml_text(light,"cast_shadows","true")
    xml_text(light,"pose","0 0 10 0 0 0")
    xml_text(light,"diffuse",".95 .95 .95 1")
    xml_text(light,"specular",".4 .4 .4 1")
    xml_text(light,"direction","-.2 -.4 -1")
    box(world,"floor",(0,0,-.06),(12,12,.12),".55 .57 .62 1")
    for i,(x,y,w,h) in enumerate(WALLS):
        box(world,f"wall_{i}",(x,y,.75),(w,h,1.5),".75 .8 .87 1")
    box(world,"blue_toolbox",(-3,-3,.18),(.32,.27,.36),".02 .08 .95 1")
    box(world,"red_first_aid",(-3,3,.20),(.34,.25,.40),".96 .04 .03 1")
    path.write_text('<?xml version="1.0"?>\n'+tostring(sdf,encoding="unicode"),encoding="utf-8")


def occupied(x,y):
    for cx,cy,w,h in WALLS:
        if abs(x-cx)<=w/2+.06 and abs(y-cy)<=h/2+.06:
            return True
    return False


def generate(destination:Path,resolution=.1):
    if not 0.05<=resolution<=.25:
        raise ValueError("Resolution must be 0.05..0.25 meters")
    destination.mkdir(parents=True,exist_ok=True)
    make_world(destination/"room_world.sdf.xacro")
    n=round(12/resolution)
    with (destination/"map.pgm").open("w",encoding="ascii") as target:
        target.write(f"P2\n# SpatialMind aligned occupancy map\n{n} {n}\n255\n")
        for py in range(n-1,-1,-1):
            y=-6+(py+.5)*resolution
            row=[]
            for px in range(n):
                x=-6+(px+.5)*resolution
                row.append("0" if occupied(x,y) else "254")
            target.write(" ".join(row)+"\n")
    (destination/"map.yaml").write_text(
        f'image: map.pgm\nmode: trinary\nresolution: {resolution}\n'
        'origin: [-6.0, -6.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n',
        encoding="utf-8")
    semantic={
        "version":"generated-room-world-v1",
        "calibrated":True,
        "frame_id":"map",
        "rooms":{name:[{"x":x,"y":y,"yaw":0} for x,y in positions]
                 for name,positions in ROOMS.items()},
    }
    (destination/"semantic_map.json").write_text(json.dumps(semantic,indent=2),encoding="utf-8")
    (destination/"manifest.json").write_text(json.dumps({
        "generator":"scripts/generate_gazebo_scene.py",
        "frame_id":"map","origin_m":[-6.,-6.],
        "resolution_m":resolution,"width_cells":n,"height_cells":n,
        "world_file":"room_world.sdf.xacro",
        "occupancy_file":"map.pgm","semantic_map":"semantic_map.json",
        "objects":["blue_toolbox","red_first_aid"],
        "note":"Generated geometry/map; Gazebo runtime still requires verification.",
    },indent=2),encoding="utf-8")
    return semantic


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--out",default="simulation/generated")
    parser.add_argument("--resolution",type=float,default=.1)
    args=parser.parse_args()
    generate(Path(args.out),args.resolution)
    print("Generated Gazebo world, occupancy map and semantic waypoints:",args.out)


if __name__=="__main__":
    main()
