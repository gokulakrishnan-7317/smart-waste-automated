@echo off
title 6 - Test ESP32 DevKit Motors & Arm Servos
cd /d "%~dp0"
echo =========================================================================
echo   ESP32 DevKit Hardware Diagnostics (Motors & 6-DOF Robotic Arm)
echo   Target: UDP 192.168.137.204:8888
echo =========================================================================
echo.
python tests/test_devkit_control.py
pause
