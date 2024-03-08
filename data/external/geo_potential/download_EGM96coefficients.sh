#!/bin/bash
cd `dirname $0`

# #establish anonymous ftp connection and download the file
# ftp -n -v -A<<END
# open cddis.gsfc.nasa.gov
# user ftp test@
# lcd $DIR_GEOPOTENTIAL
# cd /pub/egm96/general_info
# bin
# get egm96_to360.ascii
# quit
# END

# curl ftp://cddis.gsfc.nasa.gov/pub/egm96/general_info/egm96_to360.ascii > egm96_to360.ascii
