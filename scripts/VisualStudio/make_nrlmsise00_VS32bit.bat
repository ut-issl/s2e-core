@echo off
rem NRLMSISE00 library for Visual Studio 32bit
title NRLMSISE00 library builder
cd /d %~dp0

:VER_CHECK
echo.
set /P VS_version=Please chose your VS version (2017 or 2019): 
IF "%VS_version%" == "2017" (
  echo version = "%VS_version%"
) ELSE IF "%VS_version%" == "2019" (
  echo version = "%VS_version%"
) ELSE (
  echo The version %VS_version% is not supported by this script.
  echo Please compile the NRLMSISE library manually.
  GOTO :VER_CHECK
)

:EDI_CHECK
echo.
set /P VS_edition=Please chose your VS edition (P/Professional, E/Enterprise, or C/Community): 
IF /i "%VS_edition%" == "Professional" (
  set VS_edition=Professional
) ELSE IF /i "%VS_edition%" == "P" (
  set VS_edition=Professional
) ELSE IF /i "%VS_edition%" == "Enterprise" (
  set VS_edition=Enterprise
) ELSE IF /i "%VS_edition%" == "E" (
  set VS_edition=Enterprise
) ELSE IF /i "%VS_edition%" == "Community" (
  set VS_edition=Community
) ELSE IF /i "%VS_edition%" == "C" (
  set VS_edition=Community
) ELSE (
  echo The edition %VS_edition% is not supported by this script.
  echo Please compile the NRLMSISE library manually.
  GOTO :EDI_CHECK
)
echo edition = "%VS_edition%"

echo.
echo If settings are OK, press any key. A new window will open...
pause >nul

rem modify the following path to suit with your environment if you need
start %comspec% /k "C:\Program Files (x86)\Microsoft Visual Studio\%VS_version%\%VS_edition%\VC\Auxiliary\Build\vcvars32.bat"

color 0c
echo.
echo Execute "make_libnrlmsise.bat" in new command prompt window to compile NRLMSISE00
echo If new command prompt for Visual Studio Compile "is not opened", please check the path for vcvars32.bat file, and change the script file.
TIMEOUT /T 60

:FIN_CHECK
set /P fin_input=After finishing the compiling please input "finish" or "f" here: 
IF /i "%fin_input%" == "finish" GOTO :END
IF /i "%fin_input%" == "f" GOTO :END
GOTO :FIN_CHECK

echo.
echo Complete!
echo Press any key to exit...
pause >nul

