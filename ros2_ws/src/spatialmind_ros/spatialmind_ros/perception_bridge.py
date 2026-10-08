"""RGB-D -> calibrated map-frame object observations, without simulator truth."""
from __future__ import annotations

import json
import math
import struct
import time

from spatialmind.vision import (
    CameraModel, detect_color_regions, project_optical_bbox,
    visible_optical_depth_cells,
)


def rgb8_bytes(message):
    if message.encoding not in ("rgb8","bgr8"):
        raise ValueError("RGB image must be rgb8 or bgr8")
    out=bytearray(message.width*message.height*3)
    for y in range(message.height):
        row=message.data[y*message.step:y*message.step+message.width*3]
        if len(row)!=message.width*3:
            raise ValueError("Truncated RGB row")
        for x in range(message.width):
            i=3*(y*message.width+x)
            r,g,b=row[3*x:3*x+3]
            if message.encoding=="bgr8":
                r,b=b,r
            out[i:i+3]=bytes((r,g,b))
    return bytes(out)


def depth_float_m(message):
    if message.encoding not in ("16UC1","32FC1"):
        raise ValueError("Depth encoding must be 16UC1 or 32FC1")
    size=2 if message.encoding=="16UC1" else 4
    order=">" if message.is_bigendian else "<"
    code="H" if size==2 else "f"
    values=[]
    for y in range(message.height):
        for x in range(message.width):
            offset=y*message.step+x*size
            if offset+size>len(message.data):
                raise ValueError("Truncated depth image")
            value=struct.unpack_from(order+code,message.data,offset)[0]
            values.append(value/1000.0 if size==2 else float(value))
    return values


def build_payload(
    rgb,depth,info,translation_xyz,quaternion_xyzw,
    *, frame_id="map",coverage_floor=0.75,
):
    if (rgb.width,rgb.height)!=(depth.width,depth.height):
        raise ValueError("RGB and depth must be pixel-aligned")
    if info.width!=rgb.width or info.height!=rgb.height:
        raise ValueError("Camera calibration mismatch")
    camera=CameraModel(info.width,info.height,float(info.k[0]),
                       float(info.k[4]),float(info.k[2]),float(info.k[5]),
                       rgb.header.frame_id)
    ts=time.time()
    depths=depth_float_m(depth)
    candidates=detect_color_regions(rgb8_bytes(rgb),rgb.width,rgb.height)
    detected=[]
    for n,box in enumerate(candidates):
        result=project_optical_bbox(
            box,depths,camera,translation_xyz,quaternion_xyzw,stamp=ts,
            map_frame=frame_id,
            evidence_ref=f"rgbd:{rgb.header.stamp.sec}.{rgb.header.stamp.nanosec}:{n}",
        )
        if result:
            detected.append({
                "label":result.label,"x":result.pose.x,"y":result.pose.y,
                "variance":result.pose.position_variance,
                "confidence":result.confidence,"evidence_ref":result.evidence_ref,
            })
    cells=visible_optical_depth_cells(
        depths,camera,translation_xyz,quaternion_xyzw,resolution=.25)
    valid=sum(1 for d in depths if math.isfinite(d) and .1<d<8)
    fraction=valid/max(1,len(depths))
    quality=min(.95,fraction) if fraction>=coverage_floor else 0.0
    return {
        "frame_id":frame_id,"stamp":ts,
        "sensor_stamp":{"sec":rgb.header.stamp.sec,
                        "nanosec":rgb.header.stamp.nanosec},
        "visible_cells":[list(p) for p in sorted(cells)],
        "coverage_quality":quality,
        "detections":detected,"detector":"cpu-color-connected-components",
    }


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo
    from std_msgs.msg import String
    from message_filters import Subscriber,ApproximateTimeSynchronizer
    from tf2_ros import Buffer,TransformListener

    class RGBDBridge(Node):
        def __init__(self):
            super().__init__("spatialmind_rgbd_bridge")
            self.declare_parameter("rgb_topic","/rgbd_camera/image")
            self.declare_parameter("depth_topic","/rgbd_camera/depth_image")
            self.declare_parameter("info_topic","/rgbd_camera/camera_info")
            self.declare_parameter("map_frame","map")
            self.declare_parameter("slop_s",.13)
            self.declare_parameter("coverage_floor",.75)
            self.tf_buffer=Buffer()
            self.listener=TransformListener(self.tf_buffer,self)
            self.publisher=self.create_publisher(
                String,"/spatialmind/observation_json",10)
            self.rgb_sub=Subscriber(
                self,Image,self.get_parameter("rgb_topic").value,
                qos_profile=qos_profile_sensor_data)
            self.depth_sub=Subscriber(
                self,Image,self.get_parameter("depth_topic").value,
                qos_profile=qos_profile_sensor_data)
            self.info_sub=Subscriber(
                self,CameraInfo,self.get_parameter("info_topic").value,
                qos_profile=qos_profile_sensor_data)
            self.sync=ApproximateTimeSynchronizer(
                [self.rgb_sub,self.depth_sub,self.info_sub],12,
                self.get_parameter("slop_s").value)
            self.sync.registerCallback(self.on_frames)
            self.get_logger().info("SpatialMind RGB-D ready; waiting for TF2 + synchronized frames")

        def on_frames(self,rgb,depth,info):
            try:
                tf=self.tf_buffer.lookup_transform(
                    self.get_parameter("map_frame").value,rgb.header.frame_id,
                    Time.from_msg(rgb.header.stamp),timeout=Duration(seconds=.1))
                tr=tf.transform.translation
                q=tf.transform.rotation
                payload=build_payload(
                    rgb,depth,info,(tr.x,tr.y,tr.z),(q.x,q.y,q.z,q.w),
                    frame_id=self.get_parameter("map_frame").value,
                    coverage_floor=self.get_parameter("coverage_floor").value)
                message=String()
                message.data=json.dumps(payload,separators=(",",":"))
                self.publisher.publish(message)
            except Exception as exc:
                self.get_logger().warning(f"Skipping RGB-D frame: {type(exc).__name__}: {exc}")

    rclpy.init()
    node=RGBDBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
