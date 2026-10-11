@echo off
title 3 - Test YOLO Detection (ESP32-CAM Stream)
cd /d "%~dp0"
echo =========================================================================
echo   Testing AI Waste Detection on ESP32-CAM Wireless Stream
echo   Model: models/best_trash_yolov8m.pt (YOLOv8m - 12 Classes)
echo   GPU: NVIDIA GeForce RTX 3050 (CUDA)
echo =========================================================================
echo.
python tests/test_manual_ml_cam.py %*
pause
