@echo off
rem CSPICE library downloader for Visual Studio 32bit
title CSPICE library downloader
cd /d %~dp0

rem variables
set DIR_TMP=.\tmp_cspice\
set DIR_CSPICE=..\..\..\ExtLibraries\cspice\
set URL_CSPICE=http://naif.jpl.nasa.gov/pub/naif/toolkit/C/PC_Windows_VisualC_32bit/packages/cspice.zip
set URL_KERNEL=https://naif.jpl.nasa.gov/pub/naif/generic_kernels/

rem make directory
mkdir %DIR_CSPICE%
mkdir %DIR_TMP%
mkdir %DIR_TMP%\generic_kernels\lsk\
mkdir %DIR_TMP%\generic_kernels\pck\
mkdir %DIR_TMP%\generic_kernels\spk\planets\

rem download cspice lib for Visual Studio 32bit
bitsadmin /transfer download %URL_CSPICE% %CD%%DIR_TMP%cspice.zip
rem unzip
powershell expand-archive -Path %DIR_TMP%cspice.zip -DestinationPath %DIR_TMP%
rem move include files
xcopy %DIR_TMP%cspice\include %DIR_CSPICE%include\

rem download kernel files
bitsadmin /transfer download %URL_KERNEL%lsk/a_old_versions/naif0010.tls %CD%%DIR_TMP%\generic_kernels\lsk\naif0010.tls
bitsadmin /transfer download %URL_KERNEL%pck/de-403-masses.tpc %CD%%DIR_TMP%\generic_kernels\pck\de-403-masses.tpc
bitsadmin /transfer download %URL_KERNEL%pck/gm_de431.tpc %CD%%DIR_TMP%\generic_kernels\pck\gm_de431.tpc
bitsadmin /transfer download %URL_KERNEL%pck/pck00010.tpc %CD%%DIR_TMP%\generic_kernels\pck\pck00010.tpc
bitsadmin /transfer download %URL_KERNEL%spk/planets/de430.bsp %CD%%DIR_TMP%\generic_kernels\spk\planets\de430.bsp
rem move kernel files
xcopy /e %DIR_TMP%generic_kernels %DIR_CSPICE%generic_kernels\

rem compile cspice
cd %DIR_TMP%cspice\

:VER_CHECK
echo.
set /P VS_version=Please chose your VS version (2017 or 2019): 
IF "%VS_version%" == "2017" (
  echo version = "%VS_version%"
) ELSE IF "%VS_version%" == "2019" (
  echo version = "%VS_version%"
) ELSE (
  echo The version %VS_version% is not supported by this script.
  echo Please compile the CSPICE library manually.
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
  echo Please compile the CSPICE library manually.
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
echo Execute "makeall.bat" in new command prompt window to compile CSPICE
echo If new command prompt for Visual Studio Compile "is not opened", please check the path for vcvars32.bat file, and change the script file.
TIMEOUT /T 60

:FIN_CHECK
set /P fin_input=After finishing the compiling please input "finish" or "f" here: 
IF /i "%fin_input%" == "finish" GOTO :END
IF /i "%fin_input%" == "f" GOTO :END
GOTO :FIN_CHECK

:END
color 07
echo Start copy lib

cd ..\..\
rem move lib files
xcopy %DIR_TMP%cspice\lib %DIR_CSPICE%cspice_msvs\lib\

rem delete tmp files
echo.
echo Delete temporary files
echo Before input "Yes", please close the command prompt for Visual Studio
rd /s %DIR_TMP%

echo.
echo Complete!
echo Press any key to exit...
pause >nul
