@echo off
title Web 3D Digital Twin - Packaging Line Simulation
cd /d "%~dp0"
echo ======================================================================
echo   WEB 3D DIGITAL TWIN - HE THONG DONG GOI ROBOT DELTA ^& 6-DOF ARM
echo ======================================================================
echo Dang mo trinh duyet va khoi chay Server tai cong 8080...
echo.
timeout /t 1 >nul
start "" "http://localhost:8080"
python -m http.server 8080
pause
