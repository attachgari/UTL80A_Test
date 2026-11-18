@echo off
setlocal
set "tcnTool=%~dp0Tool"
if exist "%~dp0..\..\Research\" if exist "%~dp0..\..\Output\" set "tcnTool=%~dp0..\..\Tool"
if not exist "%tcnTool%\" mkdir "%tcnTool%"
pushd "%tcnTool%" || exit /b 1
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0Start-UTL80A.ps1"
set "result=%ERRORLEVEL%"
popd
if not "%result%"=="0" pause
exit /b %result%
