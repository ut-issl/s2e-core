#!/bin/bash
cd `dirname $0`

#set variables
DIR_TMP=../../../ExtLibraries/tmp_hipcatalogue/
DIR_HIPCATALOGUE=../../../ExtLibraries/HipparcosCatalogue/

mkdir -p $DIR_TMP
mkdir -p $DIR_HIPCATALOGUE

# #establish anonymous ftp connection and download the file
# ftp -n -v -A<<END
# open cdsarc.u-strasbg.fr
# user ftp test@
# lcd ../../../ExtLibraries/tmp_hipcatalogue
# cd /pub/cats/I/239
# bin
# get hip_main.dat.gz
# quit
# END

curl ftp://cdsarc.u-strasbg.fr/pub/cats/I/239/hip_main.dat > $DIR_TMP/hip_main.dat

#unzip the downloaded file
#gzip -d -v $DIR_TMP/hip_main.dat.gz

#extract reqired datum and put them into the csv file
cut -f 2,6,9,10 -d "|" $DIR_TMP/hip_main.dat>$DIR_TMP/tmp.csv

#edit the delimeter from  '|' to ','
find $DIR_TMP/tmp.csv | xargs sed -i "s/|/,/g"

#remove spaces
find $DIR_TMP/tmp.csv | xargs sed -i "s/[ ]//g"

#remove some datum which have no information about RA and DE
find $DIR_TMP/tmp.csv | xargs sed -i "/,,/d"

#sort datum from the smallest Vmag to largest Vmag
sort -t, -k2n $DIR_TMP/tmp.csv>$DIR_HIPCATALOGUE/hip_main.csv

#add header
HEADER=HIP_ID[],VMAG[],RA[deg],DE[deg]
find $DIR_HIPCATALOGUE/hip_main.csv | xargs sed -i "1i$HEADER"

#remove tmp directry
rm -rf $DIR_TMP
