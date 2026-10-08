"""CPU-only RGB-D projection, simple color perception and measured visibility.

This is a transparent detector baseline, NOT an open-vocabulary VLM.
Real camera calibration and synchronized TF must be provided upstream.
"""
from __future__ import annotations

import math
import statistics
from dataclasses import dataclass

from spatialmind.physical import ObjectEstimate, Pose2D


@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    frame_id: str = "camera_optical_frame"

    def __post_init__(self):
        if self.width < 1 or self.height < 1 or min(self.fx, self.fy) <= 0:
            raise ValueError("Invalid camera calibration")


@dataclass(frozen=True)
class BoundingBox:
    x0: int
    y0: int
    x1: int
    y1: int
    label: str
    confidence: float = 1.0

    def __post_init__(self):
        if self.x0 >= self.x1 or self.y0 >= self.y1:
            raise ValueError("Empty bounding box")


class RGBDProjector:
    """Project *measured* optical-frame depth to planar map coordinates.

    map_camera is camera location + heading of optical forward axis in
    world XY; positive pixel-u is robot-right. This is a planar-camera
    approximation and requires the robot camera to be aligned horizontally.
    """
    def __init__(self, intrinsics: CameraModel, max_range_m: float = 8.0):
        self.k = intrinsics
        if max_range_m <= 0:
            raise ValueError("max_range_m must be positive")
        self.max_range_m = max_range_m

    def project(self, box: BoundingBox, depths: list[float],
                map_camera: Pose2D, evidence: str | None = None) -> ObjectEstimate | None:
        if len(depths) != self.k.width * self.k.height:
            raise ValueError("Depth image shape does not match CameraInfo")
        x0 = max(0, box.x0)
        x1 = min(self.k.width, box.x1)
        y0 = max(0, box.y0)
        y1 = min(self.k.height, box.y1)
        if x0 >= x1 or y0 >= y1:
            return None
        # Only use inner ROI to suppress contaminated background pixels.
        margin_x, margin_y = (x1-x0)//4, (y1-y0)//4
        pixel_depths: list[float] = []
        for v in range(y0+margin_y, max(y0+margin_y+1, y1-margin_y)):
            for u in range(x0+margin_x, max(x0+margin_x+1, x1-margin_x)):
                d = depths[v*self.k.width + u]
                if math.isfinite(d) and 0.05 < d < self.max_range_m:
                    pixel_depths.append(d)
        if len(pixel_depths) < 2:
            return None
        z = statistics.median(pixel_depths)
        u_center = ((x0+x1) * 0.5 - self.k.cx) * z / self.k.fx
        # optical x: positive robot-right; optical z: forward
        yaw = map_camera.yaw
        x = map_camera.x + z*math.cos(yaw) - u_center*math.sin(yaw)
        y = map_camera.y + z*math.sin(yaw) + u_center*math.cos(yaw)
        spread = statistics.pvariance(pixel_depths) if len(pixel_depths) > 1 else 0
        variance = max(0.0025, spread + map_camera.position_variance)
        return ObjectEstimate(
            box.label, Pose2D(x, y, 0, map_camera.frame_id,
                              map_camera.stamp, variance),
            box.confidence, evidence_ref=evidence)


def visible_depth_cells(
    robot: Pose2D,
    depths: list[float],
    k: CameraModel,
    *, resolution: float = 0.25, max_range_m: float = 5,
    angular_stride: int = 4,
) -> frozenset[tuple[int, int]]:
    """Ray-trace *measured free space* to each valid depth pixel sample.

    Coverage never extends past the observed depth endpoint. High-confidence
    negative evidence must additionally check time sync, occlusion, FOV and
    detector recall; cells alone are not proof of target absence.
    """
    if len(depths) != k.width*k.height:
        raise ValueError("Depth shape mismatch")
    if resolution <= 0 or angular_stride < 1:
        raise ValueError("Invalid visibility parameters")
    cells: set[tuple[int, int]] = set()
    mid_v = k.height//2
    for u in range(0, k.width, angular_stride):
        depth = depths[mid_v*k.width+u]
        if not math.isfinite(depth) or not 0.1 < depth < max_range_m:
            continue
        angle = robot.yaw + math.atan2((u-k.cx), k.fx)
        # Exclude the obstacle endpoint from verified free space.
        step_count = max(0, int((depth-0.1)/resolution))
        for step in range(step_count):
            distance = step*resolution
            pose = Pose2D(robot.x+distance*math.cos(angle),
                          robot.y+distance*math.sin(angle),
                          frame_id=robot.frame_id, stamp=robot.stamp)
            cells.add(pose.cell(resolution))
    return frozenset(cells)


def detect_color_regions(
    rgb: bytes, width: int, height: int, *,
    min_pixels: int = 8,
) -> list[BoundingBox]:
    """Transparent red/blue connected-component baseline for demo targets.

    Input is packed RGB8. No simulator object ground-truth APIs are used.
    Works for controlled visual markers; NOT robust real-world semantics.
    """
    if len(rgb) != width*height*3:
        raise ValueError("Expected packed rgb8 image")
    if min_pixels < 1:
        raise ValueError("min_pixels must be positive")
    classes: dict[str, set[int]] = {"blue toolbox":set(), "red first aid kit":set()}
    for i in range(width*height):
        r,g,b = rgb[3*i:3*i+3]
        if b > 110 and b > 1.55*r and b > 1.35*g:
            classes["blue toolbox"].add(i)
        if r > 120 and r > 1.55*g and r > 1.55*b:
            classes["red first aid kit"].add(i)
    detections=[]
    for label, pixels in classes.items():
        visited:set[int]=set()
        for idx in sorted(pixels):
            if idx in visited:
                continue
            queue=[idx]
            visited.add(idx)
            component=[]
            while queue:
                current=queue.pop()
                component.append(current)
                x,y=current%width,current//width
                for px,py in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
                    neighbor=py*width+px
                    if 0 <= px < width and 0 <= py < height and neighbor in pixels and neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)
            if len(component)>=min_pixels:
                xs=[p%width for p in component]
                ys=[p//width for p in component]
                detections.append(BoundingBox(min(xs),min(ys),
                                              max(xs)+1,max(ys)+1,label,0.9))
    return detections
