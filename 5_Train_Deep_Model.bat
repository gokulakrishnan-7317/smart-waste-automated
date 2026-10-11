@echo off
title 5 - Deep YOLOv8m Waste Model Trainer
cd /d "%~dp0"
echo =========================================================================
echo   Deep Neural Network Trainer (YOLOv8m - 12 Classes)
echo   GPU: NVIDIA GeForce RTX 3050 (CUDA)
echo   Dataset: dataset/data.yaml (2,267 Images)
echo =========================================================================
echo.
python scripts/train_deep_model.py %*
pause
