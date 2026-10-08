"""Optional ROS1/ROS2 bag → registered RGB-D PNG sequence export.

Requires explicit topics and depth scale. Uses *actual* serialized Image data,
never simulator object truth. Designed for OpenLORIS exports and user ROS bags.
"""
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path


def image_to_pillow(msg, *, color: bool):
    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError('Pillow required: pip install -e ".[datasets]"') from exc
    width, height = int(msg.width), int(msg.height)
    step = int(msg.step)
    raw = bytes(msg.data)
    if width <= 0 or height <= 0 or step <= 0 or len(raw) < height * step:
        raise ValueError("Invalid ROS image buffer dimensions")
    encoding = str(msg.encoding).lower()
    if color:
        if encoding not in ("rgb8", "bgr8"):
            raise ValueError("RGB topic must be uncompressed rgb8/bgr8; decompress upstream")
        return Image.frombytes("RGB", (width, height), raw, "raw",
                               "RGB" if encoding == "rgb8" else "BGR", step, 1)
    if encoding not in ("16uc1", "mono16"):
        raise ValueError("Depth topic must be uncompressed 16UC1/mono16")
    # Pillow does not accept I;16L as a raw decoder for mode I;16 in all
    # versions. Select the native mode and matching decoder explicitly.
    bit_order = "I;16B" if bool(msg.is_bigendian) else "I;16"
    return Image.frombytes(bit_order, (width, height), raw, "raw", bit_order, step, 1)


@dataclass
class Frame:
    stamp: float
    image: object


def export_rosbag(
    bag: str | Path, output: str | Path, *,
    rgb_topic: str, depth_topic: str, camera_info_topic: str | None = None,
    depth_scale_to_m: float, max_frames: int = 1000,
    max_delta_s: float = 0.04,
) -> dict:
    if not rgb_topic or not depth_topic or rgb_topic == depth_topic:
        raise ValueError("Provide two distinct RGB/depth ROS topics")
    if depth_scale_to_m <= 0 or max_frames < 1 or max_delta_s <= 0:
        raise ValueError("Invalid scale, frame count or time tolerance")
    bag = Path(bag)
    if not bag.exists():
        raise FileNotFoundError(bag)
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as exc:
        raise RuntimeError('Install rosbags: pip install -e ".[datasets-rosbag]"') from exc
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "rgb").mkdir(exist_ok=True)
    (output / "depth").mkdir(exist_ok=True)
    rgb_pending: list[Frame] = []
    depth_pending: list[Frame] = []
    pairs: list[tuple[Frame, Frame]] = []
    camera = None
    topics = {rgb_topic, depth_topic}
    if camera_info_topic:
        topics.add(camera_info_topic)
    with AnyReader([bag]) as reader:
        connections = [c for c in reader.connections if c.topic in topics]
        available = {c.topic for c in connections}
        missing = {rgb_topic, depth_topic} - available
        if missing:
            raise ValueError(f"Missing ROS topics in bag: {sorted(missing)}")
        for connection, timestamp, raw in reader.messages(connections=connections):
            stamp = timestamp / 1e9
            msg = reader.deserialize(raw, connection.msgtype)
            if camera_info_topic and connection.topic == camera_info_topic:
                k = list(getattr(msg, "k", getattr(msg, "K", [])))
                if len(k) == 9:
                    camera = {
                        "width": int(msg.width), "height": int(msg.height),
                        "fx": float(k[0]), "fy": float(k[4]),
                        "cx": float(k[2]), "cy": float(k[5]),
                        "depth_scale_to_m": depth_scale_to_m,
                        "source": camera_info_topic,
                    }
                continue
            try:
                frame = Frame(stamp, image_to_pillow(
                    msg, color=connection.topic == rgb_topic))
            except ValueError:
                # Encodings are strict: do not silently skip unsupported image streams.
                raise
            (rgb_pending if connection.topic == rgb_topic else depth_pending).append(frame)
            if rgb_pending and depth_pending:
                closest = min(
                    ((abs(a.stamp-b.stamp), i, j) for i, a in enumerate(rgb_pending)
                     for j, b in enumerate(depth_pending)),
                    key=lambda x: x[0],
                )
                if closest[0] <= max_delta_s:
                    _, i, j = closest
                    pairs.append((rgb_pending.pop(i), depth_pending.pop(j)))
            rgb_pending = rgb_pending[-8:]
            depth_pending = depth_pending[-8:]
            if len(pairs) >= max_frames:
                break
    if not pairs:
        raise ValueError("No synchronized RGB/depth pairs extracted")
    rgb_index, depth_index = [], []
    for i, (rgb, depth) in enumerate(pairs):
        rgb_name = f"rgb/{i:06d}.png"
        depth_name = f"depth/{i:06d}.png"
        rgb.image.save(output / rgb_name)
        depth.image.save(output / depth_name)
        rgb_index.append(f"{rgb.stamp:.9f} {rgb_name}")
        depth_index.append(f"{depth.stamp:.9f} {depth_name}")
    (output / "color.txt").write_text("\n".join(rgb_index) + "\n")
    (output / "aligned_depth.txt").write_text("\n".join(depth_index) + "\n")
    if camera:
        (output / "camera.json").write_text(json.dumps(camera, indent=2))
    result = {
        "status": "exported_sensor_frames_not_localized",
        "frames": len(pairs), "camera_calibration_exported": bool(camera),
        "rgb_topic": rgb_topic, "depth_topic": depth_topic,
        "max_delta_s": max_delta_s,
        "note": "Verify RGB/depth registration and extrinsics. No camera/world or robot pose inferred.",
    }
    (output / "export_report.json").write_text(json.dumps(result, indent=2))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--rgb-topic", required=True)
    parser.add_argument("--depth-topic", required=True)
    parser.add_argument("--camera-info-topic")
    parser.add_argument("--depth-scale-to-m", type=float, required=True)
    parser.add_argument("--max-frames", type=int, default=1000)
    parser.add_argument("--max-delta-s", type=float, default=.04)
    args = parser.parse_args()
    result = export_rosbag(
        args.bag, args.output, rgb_topic=args.rgb_topic,
        depth_topic=args.depth_topic,
        camera_info_topic=args.camera_info_topic,
        depth_scale_to_m=args.depth_scale_to_m,
        max_frames=args.max_frames, max_delta_s=args.max_delta_s,
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
