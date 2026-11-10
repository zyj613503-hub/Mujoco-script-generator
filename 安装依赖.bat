@echo off
chcp 65001 >nul
title 安装依赖 - MuJoCo 运行器
cd /d "%~dp0"

echo ================================
echo   MuJoCo 运行器 - 安装依赖
echo ================================
echo.

set "PY="
where py >nul 2>nul && set "PY=py -3"
if not defined PY (
  where python >nul 2>nul && set "PY=python"
)
if not defined PY (
  echo [错误] 没有找到 Python。
  echo.
  echo 请先安装 Python 3.10 或更高版本（64 位）：
  echo   https://www.python.org/downloads/windows/
  echo 安装时务必勾选 "Add python.exe to PATH"。
  echo.
  pause
  exit /b 1
)

if not exist ".venv\Scripts\python.exe" (
  echo [1/2] 正在创建虚拟环境 .venv ...
  %PY% -m venv .venv
  if errorlevel 1 (
    echo [错误] 创建虚拟环境失败，请确认 Python 安装完整。
    pause
    exit /b 1
  )
) else (
  echo [1/2] 虚拟环境已存在，跳过创建。
)

echo [2/2] 正在安装依赖 mujoco / mink / numpy（约 30 MB，请耐心等待）...
".venv\Scripts\python.exe" -m pip install --upgrade pip
".venv\Scripts\python.exe" -m pip install -r requirements.txt
if errorlevel 1 (
  echo.
  echo [错误] 依赖安装失败，请检查网络后重新双击本文件。
  pause
  exit /b 1
)

echo.
echo 校验依赖 ...
".venv\Scripts\python.exe" -c "import mujoco, mink, numpy; print('mujoco', mujoco.__version__, '| mink', mink.__version__)"
if errorlevel 1 (
  echo [错误] 依赖校验失败。
  pause
  exit /b 1
)

echo.
echo 安装完成。以后双击 start.bat 启动运行器。
pause
