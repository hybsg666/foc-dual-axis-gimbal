@echo off
cd /d "%~dp0.."
echo ================================================
echo   Build + Upload + A/B test   (about 2 minutes)
echo ================================================
echo.
D:\Anaconda3\python.exe tools\deploy_ab.py %*
echo.
pause
