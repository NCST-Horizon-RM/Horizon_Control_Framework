@echo off
rem Put this file in an STM32H7 project root directory.
rem Double click it after the project has been built.
rem Auto probe order: DAPLink first, then ST-Link.

set "FLASH_MAIN=E:\BaiduSyncdisk\strong\OpenOCD\DevEnv\H7_flash_any_project.bat"

if not exist "%FLASH_MAIN%" (
    echo [ERROR] Cannot find generic H7 flash script:
    echo %FLASH_MAIN%
    pause
    exit /b 1
)

echo Target folder: %~dp0.
echo Auto probe order: DAPLink first, then ST-Link.
echo.

if "%~1"=="" (
    call "%FLASH_MAIN%" "%~dp0."
) else if /I "%~1"=="S" (
    call "%FLASH_MAIN%" "%~dp0." S
) else if /I "%~1"=="D" (
    call "%FLASH_MAIN%" "%~dp0." D
) else (
    call "%FLASH_MAIN%" %*
)
