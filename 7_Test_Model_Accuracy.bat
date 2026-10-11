@echo off
title 7 - Validate YOLOv8 Model Accuracy Benchmarks
cd /d "%~dp0"
echo =========================================================================
echo   Benchmarking Model Accuracy Metrics on Validation Dataset
echo   Model: models/best_trash_yolov8m.pt
echo =========================================================================
echo.
python tests/test_accuracy_live.py
pause
