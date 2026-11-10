@echo off
setlocal
set "ARM_PYTHON=%USERPROFILE%\miniconda3\envs\L1-Hand-PantheraHT\python.exe"
if not exist "%ARM_PYTHON%" (
  echo Python environment missing: %ARM_PYTHON%
  echo Activate a Python environment with numpy and pin, then run the .py example directly.
  exit /b 1
)
pushd "%~dp0"
"%ARM_PYTHON%" %*
set "ARM_RESULT=%ERRORLEVEL%"
popd
exit /b %ARM_RESULT%
