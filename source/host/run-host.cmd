@echo off
setlocal
call "%~dp0activate-host.cmd"
if errorlevel 1 exit /b %ERRORLEVEL%
if "%~1"=="" (
 echo Usage: run-host.cmd script.py [arguments]
 exit /b 0
)
python %*
exit /b %ERRORLEVEL%
