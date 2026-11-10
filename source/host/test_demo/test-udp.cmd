@echo off
call "%~dp0..\run-host.cmd" "%~dp0robot_udp.py" %*
exit /b %ERRORLEVEL%
