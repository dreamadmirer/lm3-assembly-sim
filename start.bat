@echo off
chcp 65001 >nul
cd /d "%~dp0"
setlocal enabledelayedexpansion

echo ============================================
echo   LM3 人机协作装配仿真实验平台
echo ============================================
echo.

rem ---- 1) 找一个真正可用的 Python 3 ----
set "PYCMD="
python -c "import sys" >nul 2>nul && set "PYCMD=python"
if not defined PYCMD (
  py -3 -c "import sys" >nul 2>nul && set "PYCMD=py -3"
)
if not defined PYCMD (
  echo [错误] 没有找到可用的 Python 3。
  echo        请安装 Python 3 (勾选 Add Python to PATH)，或改用其他静态服务器。
  echo        也可直接访问在线版：https://dreamadmirer.github.io/lm3-assembly-sim/
  echo.
  pause
  exit /b 1
)

rem ---- 2) 选一个空闲端口（默认 8123）----
set "PORT=8123"
:findport
netstat -ano | findstr /r /c:":%PORT% .*LISTENING" >nul 2>nul
if not errorlevel 1 (
  set /a PORT+=1
  if !PORT! gtr 8140 (
    echo [错误] 8123-8140 端口都被占用，请关闭占用程序后重试。
    pause
    exit /b 1
  )
  goto findport
)

echo 目录: %cd%
echo 地址: http://127.0.0.1:%PORT%/index.html
echo.
echo 浏览器会在服务就绪后自动打开；关闭本窗口即停止服务。
echo.

rem ---- 3) 等端口真正监听后再打开浏览器（避免先开浏览器导致"无法访问"）----
start "" /b powershell -NoProfile -WindowStyle Hidden -Command ^
  "for ($i=0; $i -lt 60; $i++) { try { $c = New-Object Net.Sockets.TcpClient('127.0.0.1', %PORT%); $c.Close(); Start-Process 'http://127.0.0.1:%PORT%/index.html'; break } catch { Start-Sleep -Milliseconds 250 } }"

rem ---- 4) 启动静态服务（前台运行，Ctrl+C 停止）----
%PYCMD% -m http.server %PORT% --bind 127.0.0.1

echo.
echo 服务已停止。
pause
