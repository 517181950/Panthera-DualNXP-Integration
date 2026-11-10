@echo off
call "%~dp0env.cmd"
west build -b frdm_mcxn947/mcxn947/cpu0 -d "%~dp0build-network" "%~dp0." -- -DDTC_OVERLAY_FILE=app.overlay -DBOARD_FLASH_RUNNER=pyocd -DBOARD_DEBUG_RUNNER=pyocd
exit /b %ERRORLEVEL%
