@echo off
title 2 - Test YOLO Detection (Laptop Webcam)
cd /d "%~dp0"
echo =========================================================================
echo   Testing AI Waste Detection on Laptop Webcam
echo   Model: models/best_trash_yolov8m.pt (YOLOv8m - 12 Classes)
echo   GPU: NVIDIA GeForce RTX 3050 (CUDA)
echo =========================================================================
echo.
python tests/test_manual_ml_cam.py --source 0
pause
