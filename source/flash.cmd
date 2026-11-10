@echo off
call "%~dp0env.cmd"
west flash -d "%~dp0build" -r pyocd --tool-opt=--pack=C:/Users/KelvinLM/Documents/ClionProjects/zephyr-workspace/packs/MCXN947.pack
exit /b %ERRORLEVEL%

