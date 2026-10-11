# Smart Autonomous Waste Management Robot (AI Vision + ESP32)

An end-to-end intelligent waste detection, navigation, and robotic arm sorting system powered by **YOLOv8 Object Detection** running on GPU (Laptop / Edge PC), **ESP32-CAM** (Wireless High-Speed Video Streamer), and **ESP32 DevKit** (Wireless Motor & 6-DOF Robotic Arm Controller).

---

## 🚀 Key Features

- **Real-Time AI Vision:** Custom-trained YOLOv8 model for waste classification with RTX GPU acceleration.
- **100% Wireless Operation:** 
  - **ESP32-CAM** streams video over HTTP MJPEG (`http://<CAM_IP>:81/stream`).
  - **ESP32 DevKit** receives motion & arm control commands via high-speed UDP (`<DEVKIT_IP>:8888`).
- **3-Way Intelligent Waste Sorting:**
  - 🥬 **Bin 1 - Plastics & Cups (45°):** Bottles, cups, caps, plastic bags.
  - ♻️ **Bin 2 - Metals (135°):** Cans, batteries, pop tabs.
  - 🚯 **Bin 3 - Glass & Recyclables (0°):** Glass bottles, cardboard, paper, cartons.
- **Autonomous Navigation & Target Locking:**
  - Automated centering (deadband algorithm) and distance approach using bounding-box scaling.
  - Safety wheel halt during robotic arm pick-and-sort sequences.
- **Manual Testing & Diagnostic Modes:**
  - Standalone ML detection HUD on live camera feed.
  - Interactive motor and servo hardware diagnostics.

---

## 🛠️ System Architecture

```
                    +-----------------------+
                    |   ESP32-CAM Module    |
                    | (Streams Video @ 81)  |
                    +-----------+-----------+
                                |  HTTP Video Stream
                                v
               +---------------------------------+
               |     Laptop / AI Processing      |
               |       YOLOv8 Detection          |
               | (Visual Centering & Trajectory) |
               +----------------+----------------+
                                |  Low-Latency UDP Commands
                                v
                    +-----------------------+
                    |  ESP32 DevKit V1      |
                    | (Controls Actuators)  |
                    +-----+-----------+-----+
                          |           |
            +-------------+           +-------------+
            v                                       v
+-----------------------+               +-----------------------+
|  4x DC Motors (Chassis)               | 6-DOF Robotic Arm     |
|  via L298N Drivers    |               | via PCA9685 (I2C)     |
|  (Forward/Turn/Stop)  |               | (Base, Shoulder,      |
+-----------------------+               |  Elbow, WPitch,       |
                                        |  WRoll, Gripper Claw) |
                                        +-----------------------+
```

---

## 📌 Hardware Pinout & Wiring

### 1. ESP32 DevKit -> 2x L298N Motor Drivers (4WD / 4 Motors)
*(Keep the ENA & ENB black jumpers on both L298N boards connected)*

#### Driver 1 (Left Side: Front-Left & Rear-Left Motors)
| DevKit GPIO | L298N Driver 1 Pin | Description |
|:---|:---|:---|
| **GPIO 25** | IN1 | Front-Left Motor Forward (PWM) |
| **GPIO 26** | IN2 | Front-Left Motor Backward (PWM) |
| **GPIO 27** | IN3 | Rear-Left Motor Forward (PWM) |
| **GPIO 14** | IN4 | Rear-Left Motor Backward (PWM) |

#### Driver 2 (Right Side: Front-Right & Rear-Right Motors)
| DevKit GPIO | L298N Driver 2 Pin | Description |
|:---|:---|:---|
| **GPIO 32** | IN1 | Front-Right Motor Forward (PWM) |
| **GPIO 33** | IN2 | Front-Right Motor Backward (PWM) |
| **GPIO 18** | IN3 | Rear-Right Motor Forward (PWM) |
| **GPIO 19** | IN4 | Rear-Right Motor Backward (PWM) |

### 2. ESP32 DevKit -> PCA9685 I2C (Address 0x40)
| DevKit GPIO | PCA9685 Pin | Description |
|:---|:---|:---|
| **GPIO 21** | SDA | I2C Data Line |
| **GPIO 22** | SCL | I2C Clock Line |
| **3.3V / 5V** | VCC | Logic Power |
| **GND** | GND | Common Ground |

### 3. PCA9685 -> 6-DOF Robotic Arm Servos
| PCA Channel | Servo Joint | Description / Motion Range |
|:---|:---|:---|
| **Channel 0** | Base (MG996R) | Yaw rotation across sorting bins (0° - 180°) |
| **Channel 1** | Shoulder (MG996R) | Vertical arm elevation & reach (30° - 140°) |
| **Channel 2** | Elbow (MG996R) | Forearm pitch & extension (40° - 150°) |
| **Channel 3** | Wrist Pitch (MG90S/MG996R) | Vertical wrist tilt & angle alignment (0° - 180°) |
| **Channel 4** | Wrist Roll (MG90S) | Axial wrist rotation for grasp orientation (0° - 180°) |
| **Channel 5** | Gripper / Claw (MG90S) | Open (30°) / Clamp-Grip (95°) |

### 4. ESP32 DevKit -> Sensors & Navigation Module
| Module | Module Pin | ESP32 GPIO | Description |
|:---|:---|:---|:---|
| **HC-SR04 Ultrasonic** | `TRIG` | **GPIO 4 (D4)** | Trigger Output Pulse |
| **HC-SR04 Ultrasonic** | `ECHO` | **GPIO 23 (D23)** | Echo Pulse Input (Obstacle Guard) |
| **IR Sensor 1 (Left)** | `OUT` | **GPIO 34** | Surface / Cliff / Proximity Detection |
| **IR Sensor 2 (Right)** | `OUT` | **GPIO 35** | Surface / Cliff / Proximity Detection |
| **NEO-6M GPS** | `TX` | **GPIO 16 (RX2)** | Hardware UART2 GPS Ingest |
| **NEO-6M GPS** | `RX` | **GPIO 17 (TX2)** | Hardware UART2 GPS Transmit |
| **All Modules** | `VCC / GND` | **5V / 3.3V & GND** | Common Power & Ground |

---

## 💻 Clean Project Architecture & 1-Click Launchers

The root directory contains an interactive master launcher 
un.bat plus 8 dedicated launchers ready to execute directly:

| Launcher | Target Script | Purpose |
|:---|:---|:---|
| **
un.bat** | *Interactive Master Console* | Unified interactive CLI launcher menu (1-9 or Q). |
| **1_Start_Mission_Control.bat** | src/mission_control_server.py | Web telemetry dashboard & live camera HUD (http://localhost:8000). |
| **2_Test_Laptop_Webcam.bat** | 	ests/test_manual_ml_cam.py --source 0 | Live webcam AI test with dual-bin classification HUD. |
| **3_Test_ESP32_CAM.bat** | 	ests/test_manual_ml_cam.py | Wireless ESP32-CAM AI HUD (streams from http://<CAM_IP>:81/stream). |
| **4_Run_Smart_Waste_Robot.bat** | src/robot_wireless_controller.py | Full autonomous loop: vision centering, approach, arm pickup & sorting. |
| **5_Train_Deep_Model.bat** | scripts/train_deep_model.py | GPU YOLOv8m trainer with RTX 3050 hardware acceleration. |
| **6_Test_DevKit_Hardware.bat** | 	ests/test_devkit_control.py | Hardware test for DC drive motors and 6-DOF robotic arm servos via UDP. |
| **7_Test_Model_Accuracy.bat** | 	ests/test_accuracy_live.py | Full validation accuracy benchmark & confusion matrix generator. |
| **8_Import_Roboflow_Dataset.bat** | scripts/prepare_banana_peel_dataset.py | Drag-and-drop Roboflow ZIP/folder importer for banana peel data. |

---

## 📁 Modular Directory Architecture

`
D:\smart-waste-management-system\
├── run.bat                         # Interactive Master Mission Control Launcher
├── 1_Start_Mission_Control.bat     # Web dashboard & live AI HUD
├── 2_Test_Laptop_Webcam.bat        # Laptop webcam test
├── 3_Test_ESP32_CAM.bat            # ESP32-CAM wireless test
├── 4_Run_Smart_Waste_Robot.bat     # Full autonomous sorting robot
├── 5_Train_Deep_Model.bat          # Deep GPU YOLOv8 model training
├── 6_Test_DevKit_Hardware.bat      # Motor & 6-DOF arm diagnostics
├── 7_Test_Model_Accuracy.bat       # Model accuracy benchmark
├── 8_Import_Roboflow_Dataset.bat   # Roboflow ZIP/folder dataset importer
├── requirements.txt                # Python package dependencies
├── README.md                       # Complete Project Documentation
├── .gitignore                      # Git ignore patterns
│
├── src/                            # Core Runtime Applications
│   ├── main.py                     # Unified CLI Pipeline (train, val, predict)
│   ├── mission_control_server.py   # FastAPI backend & Web UI telemetry server
│   └── robot_wireless_controller.py# Autonomous navigation & robotic arm sorting engine
│
├── scripts/                        # Dataset & Training Tooling
│   ├── prepare_banana_peel_dataset.py # Roboflow dataset merger & validator
│   ├── train_deep_model.py         # Deep YOLOv8m GPU model trainer
│   └── train_degradable_model.py   # Secondary model fine-tuning engine
│
├── tests/                          # Test Suites & Hardware Diagnostics
│   ├── test_system.py              # 12-test automated unit test suite
│   ├── test_accuracy_live.py       # Live accuracy & mAP benchmark
│   ├── test_manual_ml_cam.py       # Live camera detection test
│   ├── test_devkit_control.py      # UDP/Serial DevKit & arm diagnostic
│   ├── test_arm_servos.py          # PCA9685 arm servo calibration
│   ├── test_bluetooth_control.py   # Bluetooth serial command test
│   ├── test_cam_connection.py      # ESP32-CAM HTTP diagnostic
│   └── view_cam.py                 # Standalone MJPEG camera stream viewer
│
├── web_ui/                         # Frontend Cockpit Interface
│   └── index.html                  # Mission Control Web Dashboard & HUD
│
├── models/                         # Trained Weights & Neural Checkpoints
│   ├── best_trash_yolov8m.pt       # Canonical multi-class model (12 Classes)
│   ├── best_degradable_yolov8.pt   # High-accuracy Banana Peel model
│   ├── roboflow_banana_weights.pt  # Auxiliary fine-tuning weights
│   └── yolov8n.pt                  # Nano base model
│
├── dataset/                        # Labeled Dataset (2,267 images)
│   ├── data.yaml                   # YOLO dataset configuration
│   ├── train/images & labels       # 1,821 training images
│   └── valid/images & labels       # 446 validation images
│
└── firmware/                       # Embedded Microcontroller Firmware
    ├── esp32_devkit_firmware/      # DevKit motors & PCA9685 6-DOF arm firmware
    ├── esp32_cam_firmware/         # ESP32-CAM MJPEG streaming firmware
    ├── arm_diagnostic/             # Direct servo diagnostic sketch
    ├── arm_direct_test/            # Arm calibration sketch
    └── i2c_scanner/                # I2C bus scanner sketch
`

---

## 🚦 Dual-Bin Sorting Logic

- **Degradable (Banana Peel - Class 11):** 
  Arm base rotates **Right 180° (Clockwise)** $\to$ Right Dustbin.
- **Non-Degradable / Recyclable (Plastics, Metals, Paper, Glass - Classes 0-10):** 
  Arm base rotates **Left 180° (Counter-Clockwise)** $\to$ Left Dustbin.
