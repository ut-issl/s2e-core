# ExtLibraries

## Overview
- S2E uses the following external libraries
  - [SPICE Toolkit](https://naif.jpl.nasa.gov/naif/toolkit.html)
    - For celestial body's information
  - [NRLMSISE00](https://www.brodo.de/space/nrlmsise/)
    - For precise air density model around the earth
  - [EGM96](https://cddis.nasa.gov/926/egm96/egm96.html)
    - For geopotential coefficient table
  - others

## Note
- Currently, we cannot access the original download page of the `egm96_to360.ascii` coefficient table file. Therefore, we decided to provide the table file in this repository.
