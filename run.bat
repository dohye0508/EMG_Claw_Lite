@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"

echo ============================================
echo  EMG Claw Lite - 실행 도우미
echo ============================================
echo.
echo [연결된 장치 목록]
py emg_claw.py --list
echo.
echo ============================================
echo.

set /p INPORT=SpikerBox 입력 COM 포트 (비워두면 자동 탐지):
set /p OUTPORT=Arduino 출력 COM 포트 (예: COM7):

if "%OUTPORT%"=="" (
    echo.
    echo [오류] Arduino 출력 포트는 반드시 입력해야 합니다.
    pause
    exit /b 1
)

if "%INPORT%"=="" (
    set CMD=py emg_claw.py --output-port %OUTPORT%
) else (
    set CMD=py emg_claw.py --input-mode serial --input-port %INPORT% --output-port %OUTPORT%
)

echo.
echo 실행: %CMD%
echo (종료하려면 Ctrl+C)
echo.
%CMD%

echo.
pause
