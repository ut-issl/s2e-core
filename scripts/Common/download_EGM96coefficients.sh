#!/bin/bash

#set variables
DIR_GEOPOTENTIAL=../../../ExtLibraries/GeoPotential

mkdir $DIR_GEOPOTENTIAL

#establish anonymous ftp connection and download the file
ftp -n -v -A<<END
open cddis.gsfc.nasa.gov
user ftp test@
lcd $DIR_GEOPOTENTIAL
cd /pub/egm96/general_info
bin
get egm96_to360.ascii
quit
END

