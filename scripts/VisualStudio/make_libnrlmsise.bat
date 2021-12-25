rem 
rem    make_libnrlmsise.bat
rem 
rem    Creates nrlmsise00.lib for MS Visual C++ 
rem

set cl= /c /O2 -D_COMPLEX_DEFINED -DMSDOS -DNON_ANSI_STDIO

cd ../../../ExtLibraries/nrlmsise00/src

cl nrlmsise-00.c nrlmsise-00_data.c

link -lib /out:libnrlmsise00.lib nrlmsise-00.obj nrlmsise-00_data.obj

move libnrlmsise00.lib ..\lib\

del *.obj

exit 0