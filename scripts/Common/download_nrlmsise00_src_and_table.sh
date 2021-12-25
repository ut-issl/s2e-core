#!/bin/bash

#set variables
DIR_NRLMSISE00=../../../ExtLibraries/nrlmsise00/
URL_NRLMSISE00=https://ccmc.gsfc.nasa.gov/pub/modelweb/atmospheric/msis/nrlmsise00/nrlmsis00_c_version/
URL_TABLE=ftp://ftp.agi.com/pub/DynamicEarthData/SpaceWeather-v1.2.txt

mkdir $DIR_NRLMSISE00
mkdir $DIR_NRLMSISE00/table/
mkdir $DIR_NRLMSISE00/src/
mkdir $DIR_NRLMSISE00/lib/


# download source and space weather table for nrlmsise00 model
curl $URL_TABLE > $DIR_NRLMSISE00/table/SpaceWeather.txt
curl $URL_NRLMSISE00/makefile > $DIR_NRLMSISE00/src/makefile
curl $URL_NRLMSISE00/nrlmsise-00.c > $DIR_NRLMSISE00/src/nrlmsise-00.c
curl $URL_NRLMSISE00/nrlmsise-00.h > $DIR_NRLMSISE00/src/nrlmsise-00.h
curl $URL_NRLMSISE00/nrlmsise-00_data.c > $DIR_NRLMSISE00/src/nrlmsise-00_data.c
curl $URL_NRLMSISE00/nrlmsise-00_test.c > $DIR_NRLMSISE00/src/nrlmsise-00_test.c

# build and make library
cd $DIR_NRLMSISE00/src
# modify compile option to 32bit
sed -i -e "/CFLAGS/s/-Wall/-m32 -Wall/" ./makefile
make
ar rcs libnrlmsise00.a nrlmsise-00.o nrlmsise-00_data.o
mv libnrlmsise00.a ../lib/libnrlmsise00.a
rm nrlmsise-00.o nrlmsise-00_data.o nrlmsise-00_test.o nrlmsise-test.exe