@echo off
cd /d "%~dp0.."
echo ================================================
echo   Encoder watch - turn the axis by hand slowly
echo ================================================
echo.
D:\Anaconda3\python.exe tools\enc_watch.py %*
echo.
pause
