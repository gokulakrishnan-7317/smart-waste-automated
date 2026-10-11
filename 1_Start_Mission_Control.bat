@echo off
title 1 - Smart Waste Mission Control Dashboard
cd /d "%~dp0"
echo =========================================================================
echo   Smart Waste AI - Mission Control Dashboard & Telemetry Server
echo   URL: http://localhost:8000
echo   Model: models/best_trash_yolov8m.pt
echo =========================================================================
echo.
start "" http://localhost:8000
python src/mission_control_server.py
pause
