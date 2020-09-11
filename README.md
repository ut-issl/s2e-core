# S2E_CORE_OSS

## Overview

- S2E(Spacecraft Simulation Environment) is a spacecraft numerical simulator developed by [ISSL](https://www.space.t.u-tokyo.ac.jp/nlab/index_e.html) at the University of Tokyo.
-  S2E can:
  - Simulate spacecraft's attitude and orbit behavior in LEO and other orbit.
  - Emulate spacecraft's components behavior not only sensors and actuators but also OBCs, power components, communications, and others.
  - Emulate embedded software inside OBCs by connecting with [C2A](https://gitlab.com/ut_issl/c2a) or others.
  - Connect with Ground Station software (not yet).
  - Output log files in CSV format.
  - Connect with real OBCs for HILS tests.
- S2E is divided a core repository and user repositories. 
  - `S2E_CORE` includes most of the functions for S2E.
  - S2E user repositories include only files and source codes to define of simulation scenario. 
- The core codes are shared with ISSL OSS members.

## Development style

- Basically, S2E developers should follow [Gitlab_settings](https://gitlab.com/ut_issl/documents/gitlab_settings). When we have our own rule of development, it will be written here.

## Documents

- Documents for S2E are summarized in [Documents_OSS](https://gitlab.com/ut_issl/s2e/documents_oss).

## How to compile

- First of all, please execute `./script/name_of_your_environment/dowload_cspice_name_of_your_environment.bat` to set up CSPICE library
  - if you use docker, excute `./script/Docker_Ubuntu/download_cspice_Linux_gcc.sh`  
    if you use OS X, excute `./script/Mac/download_cspice_Mac.sh`  
    if you use Visual Studio, excute `./script/VisualStudio/download_cspice_VS32bit.bat`
  - **Note:** The script is not completely automatic. Users need to input several simple words.
  - If the script doesn't work well in your environment, please see [How to download CSPICE library](https://gitlab.com/ut_issl/s2e/documents_oss/-/blob/master/General/HowToDwnloadCSPCElibrary.md) in [Documents_OSS](https://gitlab.com/ut_issl/s2e/documents_oss)..
- Second, please see  [How to download and make NRLMSISE00 library](https://gitlab.com/ut_issl/s2e/documents_oss/-/blob/master/General/HowToDownloadNRLMSISE00library.md) in [Documents_OSS](https://gitlab.com/ut_issl/s2e/documents_oss) and set up NRLMSISE00 library.
- To compile the CMake, please see [Documents_OSS](https://gitlab.com/ut_issl/s2e/documents_oss).

