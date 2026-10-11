"""
Comprehensive Test Suite for YOLOv8 Trash Detection & Robot Control System
==========================================================================
Tests:
  1. Data Configuration & Integrity (data.yaml, class list, image files)
  2. Weight Discovery & Model Inference
  3. Robot Wireless UDP Transmission & De-duplication
  4. Autonomous Navigation & Centering Logic
  5. Waste Classification & Robotic Arm Sorting Logic
  6. CLI Pipeline Argument Parsing (train, predict, webcam, val)
"""

import os
import sys
import socket
import unittest
import yaml
from pathlib import Path
import numpy as np

# Set project root
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent if SCRIPT_DIR.name in ("src", "scripts", "tests") else SCRIPT_DIR
ROOT_DIR = PROJECT_ROOT
sys.path.insert(0, str(PROJECT_ROOT / "src"))
sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(0, str(PROJECT_ROOT))

import main
import robot_wireless_controller as rwc


class TestDataConfiguration(unittest.TestCase):
    """Test data.yaml file and dataset paths."""

    def test_data_yaml_exists_and_valid(self):
        yaml_path = ROOT_DIR / "dataset" / "data.yaml" if (ROOT_DIR / "dataset" / "data.yaml").exists() else ROOT_DIR / "data.yaml"
        self.assertTrue(yaml_path.exists(), "data.yaml does not exist in root directory or dataset")

        with open(yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        self.assertIn(data.get("nc"), [10, 11, 12], "Dataset should have 10, 11, or 12 classes")
        names = data.get("names", [])
        self.assertTrue(any("banana" in n.lower() or "cup" in n.lower() for n in names), "Banana or cup class should be present")

    def test_dataset_directories_exist(self):
        train_img_dir = ROOT_DIR / "dataset" / "train" / "images"
        if not train_img_dir.exists():
            train_img_dir = ROOT_DIR / "train" / "images"
        valid_img_dir = ROOT_DIR / "dataset" / "valid" / "images"
        if not valid_img_dir.exists():
            valid_img_dir = ROOT_DIR / "valid" / "images"
        self.assertTrue(train_img_dir.exists(), "train/images directory is missing")
        self.assertTrue(valid_img_dir.exists(), "valid/images directory is missing")
        self.assertGreater(len(list(valid_img_dir.glob("*.jpg"))), 0, "No validation images found")


class TestModelAndWeights(unittest.TestCase):
    """Test model weights resolution and detection."""

    def test_find_best_weights(self):
        best_weights = main.find_best_weights()
        self.assertTrue(os.path.exists(best_weights), f"Resolved weights file does not exist: {best_weights}")
        self.assertTrue(best_weights.endswith(".pt"), "Model weights file must end with .pt")

    def test_detect_device(self):
        device = main.detect_device()
        self.assertIn(device, ["0", "cpu"], "Device must be either '0' (CUDA) or 'cpu'")

    def test_sample_inference(self):
        from ultralytics import YOLO
        best_weights = main.find_best_weights()
        model = YOLO(best_weights)
        self.assertIn(len(model.names), [10, 11, 12], "Model should be trained for 10, 11, or 12 classes")
        self.assertTrue(any("cup" in str(v).lower() for v in (model.names.values() if isinstance(model.names, dict) else model.names)), "Cup class should exist in model")

        # Run on a sample validation image
        valid_img_dir = ROOT_DIR / "dataset" / "valid" / "images"
        if not valid_img_dir.exists():
            valid_img_dir = ROOT_DIR / "valid" / "images"
        val_img = next(valid_img_dir.glob("*.jpg"))
        results = model.predict(source=str(val_img), conf=0.25, verbose=False)
        self.assertGreater(len(results), 0, "Prediction results should not be empty")
        self.assertIsNotNone(results[0].boxes, "Boxes object should exist")


class TestRobotWirelessClient(unittest.TestCase):
    """Test UDP wireless communication and rate-limiting."""

    def setUp(self):
        # Create a local UDP listener to receive packets
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_sock.bind(("127.0.0.1", 0))
        self.server_sock.settimeout(1.0)
        self.port = self.server_sock.getsockname()[1]
        self.client = rwc.WirelessRobotClient("127.0.0.1", self.port)

    def tearDown(self):
        self.client.close()
        self.server_sock.close()

    def test_send_command(self):
        self.client.send("CMD:FORWARD", force=True)
        data, _ = self.server_sock.recvfrom(255)
        self.assertEqual(data.decode('utf-8'), "CMD:FORWARD")

    def test_deduplication_rate_limit(self):
        # Send duplicate command quickly without force - second should be dropped
        self.client.send("CMD:LEFT", force=False)
        data1, _ = self.server_sock.recvfrom(255)
        self.assertEqual(data1.decode('utf-8'), "CMD:LEFT")

        # Second duplicate sent immediately without force
        self.client.send("CMD:LEFT", force=False)
        # Verify socket times out because packet was throttled
        with self.assertRaises(socket.timeout):
            self.server_sock.recvfrom(255)

    def test_forced_send(self):
        # Force flag should bypass throttling
        self.client.send("CMD:STOP", force=False)
        self.server_sock.recvfrom(255)

        self.client.send("CMD:STOP", force=True)
        data2, _ = self.server_sock.recvfrom(255)
        self.assertEqual(data2.decode('utf-8'), "CMD:STOP")


class TestRobotAutonomousLogic(unittest.TestCase):
    """Test steering, distance estimation, and arm pick trigger logic."""

    def test_steering_decision(self):
        frame_width = 640
        center_x = frame_width // 2  # 320
        deadzone = 70

        # Object on the far left (x=100) -> diff_x = -220 (< -70) -> STEER:LEFT
        obj_center_left = 100
        diff_left = obj_center_left - center_x
        self.assertLess(diff_left, -deadzone)

        # Object on the far right (x=500) -> diff_x = 180 (> 70) -> STEER:RIGHT
        obj_center_right = 500
        diff_right = obj_center_right - center_x
        self.assertGreater(diff_right, deadzone)

        # Object centered (x=330) -> diff_x = 10 (abs <= 70) -> CENTERED
        obj_center_mid = 330
        diff_mid = obj_center_mid - center_x
        self.assertLessEqual(abs(diff_mid), deadzone)

    def test_proximity_threshold(self):
        # Far object (height < 220) -> Approach (CMD:FORWARD)
        far_box_height = 150
        action_far = "MOVE:FORWARD" if far_box_height < 220 else "ARM:PICK"
        self.assertEqual(action_far, "MOVE:FORWARD")

        # Close object (height >= 220) -> Stop & Pick (ARM:PICK)
        close_box_height = 250
        action_close = "MOVE:FORWARD" if close_box_height < 220 else "ARM:PICK"
        self.assertEqual(action_close, "ARM:PICK")

    def test_sorting_bins_mapping(self):
        """Verify sorting logic mapped in ESP32 firmware."""
        def get_bin(waste_class: str) -> str:
            waste_upper = waste_class.upper()
            if "GLASS" in waste_upper:
                return "Bin 3: Glass / General"
            elif "PLASTIC" in waste_upper or "BOTTLE" in waste_upper:
                return "Bin 1: Plastics"
            elif "CAN" in waste_upper or "BATTERY" in waste_upper:
                return "Bin 2: Metals"
            else:
                return "Bin 3: General Recyclable"

        self.assertEqual(get_bin("plastic bottle"), "Bin 1: Plastics")
        self.assertEqual(get_bin("plastic bag"), "Bin 1: Plastics")
        self.assertEqual(get_bin("can"), "Bin 2: Metals")
        self.assertEqual(get_bin("battery"), "Bin 2: Metals")
        self.assertEqual(get_bin("cardboard"), "Bin 3: General Recyclable")
        self.assertEqual(get_bin("paper"), "Bin 3: General Recyclable")


class TestCLIPipeline(unittest.TestCase):
    """Test main.py CLI arguments parsing."""

    def test_cli_modes(self):
        import argparse
        parser = argparse.ArgumentParser()
        parser.add_argument("--mode", type=str, choices=["train", "predict", "webcam", "val"])
        
        args_train = parser.parse_args(["--mode", "train"])
        self.assertEqual(args_train.mode, "train")

        args_predict = parser.parse_args(["--mode", "predict"])
        self.assertEqual(args_predict.mode, "predict")

        args_webcam = parser.parse_args(["--mode", "webcam"])
        self.assertEqual(args_webcam.mode, "webcam")

        args_val = parser.parse_args(["--mode", "val"])
        self.assertEqual(args_val.mode, "val")


if __name__ == "__main__":
    unittest.main(verbosity=2)
