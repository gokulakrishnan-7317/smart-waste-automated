@echo off
title Smart Waste Management System - Master Console
cd /d "%~dp0"

:MENU
cls
color 0B
echo =========================================================================
echo       SMART WASTE MANAGEMENT SYSTEM - MISSION CONTROL LAUNCHER
echo =========================================================================
echo.
echo   [1] Start Mission Control Web UI & Telemetry Server (Port 8000)
echo   [2] Test YOLO Detection on Laptop Webcam
echo   [3] Test YOLO Detection on ESP32-CAM Wireless Stream
echo   [4] Run Smart Waste Robot (Autonomous Driving & Robotic Arm Sorting)
echo   [5] Train Deep YOLOv8m Waste Model (RTX GPU CUDA)
echo   [6] Test ESP32 DevKit Motors & 6-DOF Robotic Arm Hardware
echo   [7] Benchmark YOLO Model Accuracy Metrics on Validation Dataset
echo   [8] Import Roboflow Dataset (Banana Peel / Degradable Waste)
echo   [9] Run Automated System Test Suite (tests/test_system.py)
echo   [Q] Quit
echo.
echo =========================================================================
set /p CHOICE="Select an option (1-9 or Q): "

if "%CHOICE%"=="1" ( call 1_Start_Mission_Control.bat & goto MENU )
if "%CHOICE%"=="2" ( call 2_Test_Laptop_Webcam.bat & goto MENU )
if "%CHOICE%"=="3" ( call 3_Test_ESP32_CAM.bat & goto MENU )
if "%CHOICE%"=="4" ( call 4_Run_Smart_Waste_Robot.bat & goto MENU )
if "%CHOICE%"=="5" ( call 5_Train_Deep_Model.bat & goto MENU )
if "%CHOICE%"=="6" ( call 6_Test_DevKit_Hardware.bat & goto MENU )
if "%CHOICE%"=="7" ( call 7_Test_Model_Accuracy.bat & goto MENU )
if "%CHOICE%"=="8" ( call 8_Import_Roboflow_Dataset.bat & goto MENU )
if "%CHOICE%"=="9" (
    cls
    echo Running System Test Suite...
    python tests/test_system.py
    pause
    goto MENU
)
if /i "%CHOICE%"=="Q" exit /b 0

echo Invalid selection. Please choose 1-9 or Q.
timeout /t 2 >nul
goto MENU
