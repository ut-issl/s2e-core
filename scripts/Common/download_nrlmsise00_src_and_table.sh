#!/bin/bash
cd `dirname $0`

#set variables
DIR_NRLMSISE00=../../../ExtLibraries/nrlmsise00/
URL_NRLMSISE00=git://git.linta.de/~brodo/nrlmsise-00.git
URL_TABLE=ftp://ftp.agi.com/pub/DynamicEarthData/SpaceWeather-v1.2.txt

mkdir -p $DIR_NRLMSISE00/table/
mkdir -p $DIR_NRLMSISE00/src/
mkdir -p $DIR_NRLMSISE00/lib/


# download source and space weather table for nrlmsise00 model
curl $URL_TABLE > $DIR_NRLMSISE00/table/SpaceWeather.txt
cd $DIR_NRLMSISE00
git clone $URL_NRLMSISE00 src


# build and make library
cd src
# modify compile option to 32bit
sed -i -e "/CFLAGS/s/-Wall/-m32 -Wall/" ./makefile
make
ar rcs libnrlmsise00.a nrlmsise-00.o nrlmsise-00_data.o
mv libnrlmsise00.a ../lib/libnrlmsise00.a
# rm nrlmsise-00.o nrlmsise-00_data.o nrlmsise-00_test.o nrlmsise-test.exe
