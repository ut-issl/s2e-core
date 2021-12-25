#!/bin/bash

# set variables
DIR_CSPICE=../../../ExtLibraries/cspice
DIR_TMP=./tmp
URL_CSPICE=https://naif.jpl.nasa.gov/pub/naif/toolkit/C/PC_Linux_GCC_32bit/packages/
URL_KERNEL=https://naif.jpl.nasa.gov/pub/naif/generic_kernels/

# make directory
mkdir -p $DIR_TMP
mkdir -p $DIR_CSPICE/generic_kernels/lsk
mkdir -p $DIR_CSPICE/generic_kernels/pck
mkdir -p $DIR_CSPICE/generic_kernels/spk/planets
mkdir -p $DIR_CSPICE/cspice_unix/

# download generic kernels
curl $URL_KERNEL/lsk/a_old_versions/naif0010.tls > $DIR_CSPICE/generic_kernels/lsk/naif0010.tls
curl $URL_KERNEL/pck/de-403-masses.tpc  > $DIR_CSPICE/generic_kernels/pck/de-403-masses.tpc
curl $URL_KERNEL/pck/gm_de431.tpc  > $DIR_CSPICE/generic_kernels/pck/gm_de431.tpc
curl $URL_KERNEL/pck/pck00010.tpc  > $DIR_CSPICE/generic_kernels/pck/pck00010.tpc
curl $URL_KERNEL/spk/planets/de430.bsp  > $DIR_CSPICE/generic_kernels/spk/planets/de430.bsp

# download and unzip cspice.tar.Z
curl $URL_CSPICE/cspice.tar.Z > $DIR_TMP/cspice.tar.Z
tar -zxvf $DIR_TMP/cspice.tar.Z -C $DIR_TMP

# move include and lib directory
cp -r $DIR_TMP/cspice/include $DIR_CSPICE
cp -r $DIR_TMP/cspice/lib $DIR_CSPICE/cspice_unix/lib

# delete tmp file
rm -r $DIR_TMP