@echo off
title 4 - Smart Waste Robot Autonomous Controller
cd /d "%~dp0"
echo =========================================================================
echo   Smart Waste Robot - Autonomous Navigation ^& 6-DOF Robotic Arm Sorting
echo   Model: models/best_trash_yolov8m.pt
echo   Camera: ESP32-CAM (MJPEG)  ^|  Motors/Arm: ESP32 DevKit (UDP:8888)
echo =========================================================================
echo.
python src/robot_wireless_controller.py
pause
