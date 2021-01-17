# S2E_CORE_OSS

## Overview

- S2E(Spacecraft Simulation Environment) is a spacecraft numerical simulator developed by [ISSL](https://www.space.t.u-tokyo.ac.jp/nlab/index_e.html) at the University of Tokyo.
- S2E can:
  - Simulate spacecraft's attitude and orbit behavior in LEO and other orbits.
  - Emulate spacecraft's components behavior not only sensors and actuators but also OBCs, power components, communications, and others.
  - Emulate embedded software inside OBCs by connecting with [C2A](https://gitlab.com/ut_issl/c2a) or others.
  - Connect with Ground Station software (not yet).
  - Output log files in CSV format.
  - Connect with real OBCs for HILS tests.
- S2E is divided into a core repository and user repositories. 
  - `S2E_CORE` includes most of the functions for S2E.
  - S2E user repositories include only files and source codes to define the simulation scenario. 
- The core codes are shared with ISSL OSS members.

## Development style
- Repository settings
  - Branch structure
  ```
  .
  ├── master      # The latest operation guaranteed codes for general users
  ├── develop     # The latest buildable codes for S2E primary developers
  └── feature/*   # Developing codes
  ```
  - We recommend general users to use the `master` branch or suitable released version because we sometimes update the `develop` branch without the backward compatibility, and users need to modify their user side codes and settings.
  - Push to `master` and `develop` is prohibited. All developers have to develop with `feature/*` branch and make a merge request.
  - Maintainers confirm the request and merge it to the `develop` branch.
  - [Reference document](https://nvie.com/posts/a-successful-git-branching-model/) 

- Flow of development
  1. Make a `feature/*` branch from the `develop` branch.
  2. Edit, commit, and push in the `feature/*` branch.
  3. Make a merge request to the `develop` branch.
  4. A maintainer reviews the merge request. If some problems are found, the maintainer proposes modifications.
  5. The developer modifies the codes according to the maintainer's proposal and goes back to 3.
  6. The maintainer merges the `feature/*` branch to the `develop` branch.
  7. The ISSL's S2E development members decide to merge the `develop` branch to the `master` branch and release new version.

- Binary files
  - Binary file commit is prohibited.
  - Please write the link to such files, or make a script file to get the files.
  - **Exception**
    - Images for markdown document files are allowable when the file size is smaller than 200K Bytes.

## Documents

- Documents for S2E are summarized in [Documents_OSS](https://gitlab.com/ut_issl/s2e/documents_oss).

## How to use

- Please see [Documents_OSS](https://gitlab.com/ut_issl/s2e/documents_oss).

