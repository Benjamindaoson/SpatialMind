"""Deterministic metric navigation/perception test double for the async agent.

Hidden object placements are read ONLY by observe(), not the agent planner.
This is not Gazebo and it intentionally does not claim real sensor performance.
"""
from __future__ import annotations

import asyncio
import math
import time
from collections import deque

from spatialmind.physical import MetricObservation, MotionResult, ObjectEstimate, Pose2D, RoomMap


def demo_room_map() -> RoomMap:
    return RoomMap({
        "lab":[Pose2D(2,2),Pose2D(2,5)],
        "storage":[Pose2D(3,9),Pose2D(7,9)],
        "meeting":[Pose2D(14,2),Pose2D(17,4)],
        "office":[Pose2D(15,9),Pose2D(17,8)],
    }, "metric-grid-v1")


class MetricSimRobot:
    width,height=20,12

    def __init__(self, *, pose: Pose2D | None = None, latency_s: float = 0,
                 perception_range_m: float = 2.5):
        self._pose=pose or Pose2D(2,2)
        self.objects:dict[str,tuple[str,Pose2D]]={
            "toolbox_a":("blue toolbox",Pose2D(3,3)),
            "aid_a":("red first aid kit",Pose2D(4,9)),
        }
        self.walls={(x,0) for x in range(20)} | {(x,11) for x in range(20)}
        self.walls|={(0,y) for y in range(12)}|{(19,y) for y in range(12)}
        self.walls|={(9,y) for y in range(1,11) if y not in (3,8)}
        self.obstacles:set[tuple[int,int]]=set()
        self.known_blocked:set[tuple[int,int]]=set()
        self.latency_s=latency_s
        self.perception_range_m=perception_range_m
        self._stop=False

    @property
    def pose(self) -> Pose2D:
        return self._pose

    def place(self, object_id: str, pose: Pose2D):
        if not self._passable(pose.cell(1)):
            raise ValueError("Object outside free space")
        label,_=self.objects[object_id]
        self.objects[object_id]=(label,pose)

    def block(self, point: tuple[int,int]):
        if not self._passable(point):
            raise ValueError("Not traversable")
        self.obstacles.add(point)

    def _passable(self, point: tuple[int,int]):
        x,y=point
        return 0<=x<self.width and 0<=y<self.height and point not in self.walls

    def _path(self, start: tuple[int,int],end: tuple[int,int]):
        queue=deque([start])
        previous={start:None}
        while queue:
            current=queue.popleft()
            if current==end:
                result=[]
                while previous[current] is not None:
                    result.append(current)
                    current=previous[current]
                return list(reversed(result))
            x,y=current
            for nxt in ((x+1,y),(x,y+1),(x-1,y),(x,y-1)):
                if nxt not in previous and self._passable(nxt) and nxt not in self.known_blocked:
                    previous[nxt]=current
                    queue.append(nxt)
        return None

    async def navigate(self, target: Pose2D, *, timeout_s: float = 120) -> MotionResult:
        if target.frame_id != self.pose.frame_id:
            return MotionResult(False,"frame_mismatch")
        path=self._path(self.pose.cell(1),target.cell(1))
        if path is None:
            return MotionResult(False,"unreachable")
        started=time.monotonic()
        self._stop=False
        traveled=0
        for cell in path:
            if self._stop:
                return MotionResult(False,"cancelled",time.monotonic()-started,traveled)
            if cell in self.obstacles:
                self.known_blocked.add(cell)
                return MotionResult(False,"obstructed",time.monotonic()-started,traveled)
            if time.monotonic()-started >= timeout_s:
                return MotionResult(False,"timeout",time.monotonic()-started,traveled)
            if self.latency_s:
                await asyncio.sleep(self.latency_s)
            else:
                await asyncio.sleep(0)
            traveled+=1
            self._pose=Pose2D(cell[0],cell[1])
        return MotionResult(True,"arrived",time.monotonic()-started,traveled)

    def _clear_sight(self, target:tuple[int,int]):
        x0,y0=self.pose.cell(1)
        x1,y1=target
        dx,dy=abs(x1-x0),abs(y1-y0)
        sx=1 if x0<x1 else -1
        sy=1 if y0<y1 else -1
        err=dx-dy
        while (x0,y0)!=(x1,y1):
            e=2*err
            if e>-dy:
                err-=dy; x0+=sx
            if e<dx:
                err+=dx; y0+=sy
            if (x0,y0) in self.walls or (x0,y0) in self.obstacles:
                return False
        return True

    async def observe(self) -> MetricObservation:
        timestamp=time.time()
        pose=Pose2D(self.pose.x,self.pose.y,frame_id="map",stamp=timestamp)
        visible=frozenset((x,y) for y in range(1,11) for x in range(1,19)
                          if self._passable((x,y)) and (x,y) not in self.obstacles
                          and math.hypot(x-pose.x,y-pose.y)<=self.perception_range_m
                          and self._clear_sight((x,y)))
        detections=tuple(
            ObjectEstimate(
                label, Pose2D(loc.x,loc.y,frame_id="map",stamp=timestamp,
                              position_variance=.04),
                .95, track_id=object_id, evidence_ref=f"sim:{object_id}:{timestamp}")
            for object_id,(label,loc) in self.objects.items()
            if loc.cell(1) in visible
        )
        return MetricObservation(pose,detections,visible,coverage_quality=.95,
                                 source="metric-simulator")

    async def stop(self):
        self._stop=True
