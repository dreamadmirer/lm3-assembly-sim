@echo off
chcp 65001 >nul
cd /d "%~dp0"
setlocal enabledelayedexpansion
title LM3 Simulation Lab

echo ============================================
echo   LM3 Human-Robot Assembly Simulation Lab
echo ============================================
echo.

rem ---------- 1. find a working Python 3 ----------
set "PYCMD="
python -c "import sys" >nul 2>nul
if errorlevel 1 goto try_py
set "PYCMD=python"
goto py_ok

:try_py
py -3 -c "import sys" >nul 2>nul
if errorlevel 1 goto no_python
set "PYCMD=py -3"

:py_ok
rem ---------- 2. pick a free port ----------
set "PORT=8123"

:find_port
netstat -ano | findstr /c:":%PORT% " | findstr /c:"LISTENING" >nul 2>nul
if errorlevel 1 goto port_ok
set /a PORT+=1
if %PORT% gtr 8140 goto no_port
goto find_port

:port_ok
echo Folder : %cd%
echo URL    : http://127.0.0.1:%PORT%/index.html
echo.
echo Browser opens in 3 seconds. Keep this window open; closing it stops the server.
echo.

rem ---------- 3. open browser after a short delay ----------
start "" cmd /c "timeout /t 3 >nul & explorer http://127.0.0.1:%PORT%/index.html"

rem ---------- 4. serve (foreground) ----------
echo Starting local server...
echo.
%PYCMD% -m http.server %PORT% --bind 127.0.0.1

echo.
echo Server stopped.
pause
exit /b 0

:no_python
echo [ERROR] Python 3 not found.
echo         Install Python 3 and tick "Add Python to PATH", then retry.
echo         Online version: https://dreamadmirer.github.io/lm3-assembly-sim/
echo.
pause
exit /b 1

:no_port
echo [ERROR] Ports 8123-8140 are all in use. Close the program using them and retry.
echo.
pause
exit /b 1
