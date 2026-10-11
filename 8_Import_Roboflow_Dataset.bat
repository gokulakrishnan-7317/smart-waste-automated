@echo off
title 8 - Import Downloaded Roboflow Dataset
color 0E
cd /d "%~dp0"
echo =========================================================================
echo   ROBOFLOW DATASET IMPORTER FOR BANANA PEEL (DEGRADABLE WASTE)
echo =========================================================================
echo.
echo Drag and drop your downloaded Roboflow folder or .ZIP file here,
echo or paste its full path below (e.g. C:\Users\gokul\Downloads\banana-peel.zip)
echo.

if "%~1"=="" (
    set /p RFPATH="Path to Roboflow Folder or .ZIP: "
) else (
    set "RFPATH=%~1"
)

python scripts/prepare_banana_peel_dataset.py %RFPATH%

echo.
echo =========================================================================
echo Next step: Run '5_Train_Deep_Model.bat' to train YOLO on the new images!
echo =========================================================================
echo.
pause
