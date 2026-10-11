@echo off
chcp 65001 >nul
set PYTHONIOENCODING=utf-8
title 6 - Test ESP32 DevKit Motors & Arm Servos
cd /d "%~dp0"
echo =========================================================================
echo   ESP32 DevKit Hardware Diagnostics (Motors & 6-DOF Robotic Arm)
echo   Connection: Bluetooth Serial ("SMART_WASTE_ROBOT") / UDP 192.168.137.204
echo =========================================================================
echo.
if "%~1"=="" (
    python tests/test_devkit_control.py COM14
) else (
    python tests/test_devkit_control.py %*
)
pause
