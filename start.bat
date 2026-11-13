@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo ============================================
echo   LM3 Human-Robot Assembly Simulation Lab
echo ============================================
echo.

set PY=
where python >nul 2>nul && set PY=python
if "%PY%"=="" (where py >nul 2>nul && set PY=py -3)
if "%PY%"=="" (
  echo [ERROR] Python not found. Please install Python 3 and retry.
  pause
  exit /b 1
)

echo Serving: %cd%
echo URL:     http://localhost:8123/index.html
echo.
echo Press Ctrl+C to stop the server.
start "" http://localhost:8123/index.html
%PY% -m http.server 8123
