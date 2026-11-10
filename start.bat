@echo off
chcp 65001 >nul
title MuJoCo 运行器
cd /d "%~dp0"

set "PORT=%RUNNER_PORT%"
if "%PORT%"=="" set "PORT=8770"

set "PY="
if exist ".venv\Scripts\python.exe" set "PY=.venv\Scripts\python.exe"
if not defined PY (
  where py >nul 2>nul && set "PY=py -3"
)
if not defined PY (
  where python >nul 2>nul && set "PY=python"
)
if not defined PY (
  echo [错误] 没有找到 Python。
  echo 请先安装 Python 3.10+（64 位），安装时勾选 "Add python.exe to PATH"，
  echo 再双击「安装依赖.bat」。
  pause
  exit /b 1
)

%PY% -c "import mujoco, mink, numpy" >nul 2>nul
if errorlevel 1 (
  echo [提示] 依赖还没装好，请先双击「安装依赖.bat」。
  pause
  exit /b 1
)

echo ================================
echo   MuJoCo 运行器
echo   地址： http://127.0.0.1:%PORT%/
echo   关闭本窗口即停止服务
echo ================================
echo.

set "RUNNER_OPEN_BROWSER=1"
%PY% server\app.py
echo.
echo 服务已停止。
pause
