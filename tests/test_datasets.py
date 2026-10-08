"""Six source adapters tested with tiny local fixtures; never pretend these are real datasets."""
from __future__ import annotations
import gzip
import json
import tempfile
import unittest
from pathlib import Path

from spatialmind.datasets import DATASETS, prepare_dataset, read_records
from spatialmind.datasets.cli import main as data_main
from spatialmind.datasets.core import safe_input_file, sha256_file


class DatasetAdapterTests(unittest.TestCase):
    def setUp(self):
        self.work = tempfile.TemporaryDirectory()
        self.root = Path(self.work.name)
        self.source = self.root / "raw"
        self.source.mkdir()

    def tearDown(self):
        self.work.cleanup()

    def _json(self, path, value):
        dest = self.source / path
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text(json.dumps(value), encoding="utf-8")
        return dest

    def _prepare(self, name, **kwargs):
        out = self.root / ("prepared_" + name)
        result = prepare_dataset(name, self.source, out, **kwargs)
        records = list(read_records(out / "records.jsonl"))
        oracle = list(read_records(out / "oracle.jsonl")) if result["oracle_count"] else []
        self.assertEqual(len(records), result["record_count"])
        self.assertEqual(len(oracle), result["oracle_count"])
        self.assertEqual(result["records_sha256"], sha256_file(out / "records.jsonl"))
        self.assertTrue(all(not r.kind.startswith("oracle") for r in records))
        self.assertTrue(all(r.kind.startswith("oracle") for r in oracle))
        return result, records, oracle

    def test_registry_all_six_official_sources(self):
        self.assertEqual(set(DATASETS), {
            "tum", "openloris", "3rscan", "replicacad", "hm3d", "goat"
        })
        self.assertTrue(DATASETS["hm3d"].needs_approval)
        self.assertTrue(DATASETS["3rscan"].needs_approval)
        self.assertIn("CC BY", DATASETS["replicacad"].rights)

    def test_tum_timestamp_sync_and_real_pose(self):
        (self.source / "rgb").mkdir()
        (self.source / "depth").mkdir()
        (self.source / "rgb" / "a.png").write_bytes(b"placeholder")
        (self.source / "depth" / "a.png").write_bytes(b"placeholder")
        (self.source / "rgb.txt").write_text("1.000 rgb/a.png\n")
        (self.source / "depth.txt").write_text("1.018 depth/a.png\n")
        (self.source / "groundtruth.txt").write_text("1.001 1 2 3 0 0 0 1\n")
        _, records, _ = self._prepare("tum")
        self.assertEqual(len(records), 1)
        row = records[0].data
        self.assertAlmostEqual(row["rgb_depth_delta_s"], 0.018, places=5)
        self.assertEqual(row["camera_world_pose"]["xyz"], [1., 2., 3.])
        self.assertEqual(row["depth_scale_to_m"], 1/5000)

    def test_openloris_sensor_only_no_unsafe_camera_pose(self):
        (self.source / "rgb.png").write_bytes(b"rgb")
        (self.source / "depth.png").write_bytes(b"depth")
        (self.source / "color.txt").write_text("2.0 rgb.png\n")
        (self.source / "aligned_depth.txt").write_text("2.015 depth.png\n")
        self._json("dummy.json", {})
        _, records, _ = self._prepare("openloris")
        self.assertEqual(len(records), 1)
        self.assertIsNone(records[0].data["camera_world_pose"])
        self.assertIsNone(records[0].data["depth_scale_to_m"])

    def test_rgbd_rejects_misaligned_timestamps(self):
        (self.source / "a.png").write_bytes(b"x")
        (self.source / "b.png").write_bytes(b"x")
        (self.source / "rgb.txt").write_text("1.0 a.png\n")
        (self.source / "depth.txt").write_text("2.0 b.png\n")
        with self.assertRaises(ValueError):
            prepare_dataset("tum", self.source, self.root / "out")

    def test_3rscan_scene_and_oracle_are_separated(self):
        self._json("3RScan.json", [{
            "reference": "scan_before", "type": "train",
            "scans": [{"reference": "scan_after",
                       "transform": [1, 0, 0, 0],
                       "rigid": [{"instance_reference": 1, "instance_rescan": 1}]}],
        }])
        self._json("scan_before/semseg.v2.json", {
            "scan_id": "scan_before",
            "segGroups": [{"objectId": 1, "label": "chair",
                           "obb": {"centroid": [1, 2, 3]}}],
        })
        result, policy, oracle = self._prepare("3rscan")
        self.assertGreaterEqual(result["oracle_count"], 2)
        self.assertTrue(any(r.kind == "scan_group" for r in policy))
        self.assertTrue(any(r.kind == "oracle_scan_change" for r in oracle))
        self.assertTrue(any(r.kind == "oracle_object_instance" for r in oracle))

    def test_replicacad_scene_instances(self):
        self._json("configs/scenes/kitchen.scene_instance.json", {
            "object_instances": [{"template_name": "chair"}],
            "articulated_object_instances": [],
        })
        _, policy, _ = self._prepare("replicacad")
        self.assertEqual(policy[0].kind, "scene_asset")
        self.assertEqual(policy[0].data["stage_instance_count"], 1)

    def test_hm3d_scenes_and_objectnav_episodes(self):
        scene = self.source / "minival" / "room.basis.glb"
        scene.parent.mkdir(parents=True, exist_ok=True)
        scene.write_bytes(b"placeholder only")
        self._json("episodes/val.json", {
            "episodes": [{"episode_id": "1", "scene_id": "room",
                          "object_category": "chair",
                          "goals": [{"position": [99, 0, 99]}],
                          "start_position": [0, 0, 0]}],
            "goals_by_category": {"room_chair": [{"position": [99, 0, 99]}]},
        })
        result, policy, oracle = self._prepare("hm3d")
        self.assertEqual(result["oracle_count"], 1)
        episode = next(r for r in policy if r.kind == "navigation_episode")
        self.assertEqual(episode.data["goal_hints"][0]["object_category"], "chair")
        self.assertNotIn("position", json.dumps(episode.data.get("goal_hints")))
        self.assertEqual(oracle[0].kind, "oracle_navigation")

    def test_goat_multiple_goals_but_no_oracle_location(self):
        package = {"episodes": [{
            "episode_id": "e0", "scene_id": "hm3d_room",
            "start_position": [0, 0, 0],
            "tasks": [
                {"goal_type": "language", "description": "brown chair",
                 "position": [55, 1, 55]},
                {"goal_type": "object", "object_category": "table",
                 "position": [77, 1, 77]},
            ],
        }]}
        archive = self.source / "val_seen.json.gz"
        with gzip.open(archive, "wt", encoding="utf-8") as f:
            json.dump(package, f)
        result, policy, oracle = self._prepare("goat")
        self.assertEqual(result["oracle_count"], 1)
        self.assertEqual(policy[0].data["subtask_count"], 2)
        hints = json.dumps(policy[0].data["goal_hints"])
        self.assertNotIn("position", hints)
        self.assertIn("brown chair", hints)
        self.assertTrue(oracle[0].data["tasks"])

    def test_no_missing_dataset_is_marked_prepared(self):
        with self.assertRaises(ValueError):
            prepare_dataset("replicacad", self.source, self.root / "prepared")

    def test_path_traversal_is_rejected(self):
        outside = self.root / "secret.txt"
        outside.write_text("private")
        with self.assertRaises(ValueError):
            safe_input_file(self.source, "../secret.txt")
        with self.assertRaises(ValueError):
            safe_input_file(self.source, str(outside))
        nested = self.source / "jump"
        nested.symlink_to(outside)
        with self.assertRaises(ValueError):
            safe_input_file(self.source, "jump")

    def test_manifest_verify_command(self):
        (self.source / "frame.png").write_bytes(b"rgb")
        (self.source / "depth.png").write_bytes(b"depth")
        (self.source / "rgb.txt").write_text("1 frame.png\n")
        (self.source / "depth.txt").write_text("1 depth.png\n")
        result, _, _ = self._prepare("tum")
        check = data_main(["verify", "--manifest", str(self.root/"prepared_tum"/"manifest.json")])
        self.assertTrue(check["verified"])

    def test_local_asset_verification_detects_removed_source_file(self):
        (self.source / "frame.png").write_bytes(b"rgb")
        (self.source / "depth.png").write_bytes(b"depth")
        (self.source / "rgb.txt").write_text("1 frame.png\n")
        (self.source / "depth.txt").write_text("1 depth.png\n")
        self._prepare("tum")
        manifest = str(self.root / "prepared_tum" / "manifest.json")
        self.assertTrue(data_main(["verify", "--manifest", manifest, "--assets"])["verified"])
        (self.source / "depth.png").unlink()
        with self.assertRaises(SystemExit):
            data_main(["verify", "--manifest", manifest, "--assets"])

    def test_replay_actual_png_bytes_and_belief(self):
        from PIL import Image
        src = self.source
        (src / "rgb").mkdir()
        (src / "depth").mkdir()
        rgb = Image.new("RGB", (20, 20), (0, 0, 0))
        for y in range(5, 14):
            for x in range(5, 14):
                rgb.putpixel((x, y), (8, 10, 240))
        rgb.save(src / "rgb/f0.png")
        Image.new("I;16", (20, 20), 10000).save(src / "depth/f0.png")
        (src / "rgb.txt").write_text("1 rgb/f0.png\n")
        (src / "depth.txt").write_text("1 depth/f0.png\n")
        (src / "groundtruth.txt").write_text("1 0 0 0 0 0 0 1\n")
        self._json("camera.json", {"width": 20, "height": 20, "fx": 20,
                                   "fy": 20, "cx": 9.5, "cy": 9.5,
                                   "depth_scale_to_m": 0.0002})
        self._prepare("tum")
        from spatialmind.datasets.replay import replay_rgbd
        report = replay_rgbd(manifest=self.root/"prepared_tum"/"manifest.json",
                             camera_file=src/"camera.json",
                             output=self.root/"replay")
        self.assertEqual(report["frames_with_real_pose"], 1)
        self.assertEqual(report["detected_color_regions"], 1)
        self.assertEqual(report["map_projected_observations"], 1)
        self.assertEqual(report["memory_objects"], 1)


class ExtraDatasetIntegrationTests(unittest.TestCase):
    def test_3rscan_zip_central_directory_indexing(self):
        from zipfile import ZipFile
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scan = root / "scan-01"
            scan.mkdir()
            with ZipFile(scan / "sequence.zip", "w") as archive:
                archive.writestr("sequence/_info.txt", "m_depthShift = 1000")
                archive.writestr("sequence/frame-000000.color.jpg", b"fake-image")
                archive.writestr("sequence/frame-000000.depth.pgm", b"fake-depth")
                archive.writestr("sequence/frame-000000.pose.txt", "1 0 0 0")
            report = prepare_dataset("3rscan", root, root/"converted")
            records = list(read_records(root/"converted"/"records.jsonl"))
            self.assertEqual(report["record_count"], 2)
            frame = next(x for x in records if x.kind == "scan_rgbd_frame")
            self.assertEqual(frame.data["depth_scale_to_m"], .001)
            self.assertTrue(frame.data["pose_member"].endswith(".pose.txt"))

    def test_ros_image_codecs_preserve_uint16_and_bgr8(self):
        from spatialmind.datasets.rosbag_export import image_to_pillow
        from types import SimpleNamespace
        def message(encoding, data, step):
            return SimpleNamespace(width=2, height=1, encoding=encoding,
                                   data=data, step=step, is_bigendian=0)
        rgb = image_to_pillow(message("bgr8", bytes([200, 10, 5, 0, 5, 180]), 6),
                              color=True)
        self.assertEqual(rgb.getpixel((0, 0)), (5, 10, 200))
        depth = image_to_pillow(message("16UC1", bytes([0x88, 0x13, 0x10, 0x27]), 4),
                                color=False)
        self.assertEqual(list(depth.getdata()), [5000, 10000])

    def test_batch_pipeline_reports_missing_not_fake_success(self):
        from scripts.run_data_pipeline import run
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            config = root/"config.json"
            config.write_text(json.dumps({
                "output_root": "out", "sources": {
                    "hm3d": {"path": "missing-hm3d"},
                    "goat": {"path": "missing-goat"},
                }
            }), encoding="utf-8")
            status = run(config, selected={"hm3d", "goat"})
            self.assertEqual(status["sources"]["goat"]["status"], "missing_local_data")
            self.assertEqual(status["sources"]["hm3d"]["status"], "missing_local_data")
            with self.assertRaises(SystemExit):
                run(config, selected={"hm3d", "goat"}, fail_on_missing=True)


if __name__ == "__main__":
    unittest.main()
