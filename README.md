# S2E CORE

## Overview

- S2E (Spacecraft Simulation Environment) is a spacecraft numerical simulator developed by [ISSL](https://www.space.t.u-tokyo.ac.jp/nlab/index_e.html) at the University of Tokyo.
- S2E can:
  - Simulate spacecraft's attitude and orbit behavior in LEO and other orbits.
  - Emulate spacecraft's components behavior not only sensors and actuators but also OBCs, power components, communications, and others.
  - Emulate embedded software inside OBCs by connecting with [C2A](https://github.com/ut-issl/c2a-core) or others.
  - Connect with Ground Station software.
  - Output log files in CSV format.
  - Connect with real OBCs for HILS tests.
- S2E is divided into this core repository(`S2E CORE`) and user repositories. 
  - `S2E CORE` includes most of the functions for S2E.
  - S2E user repositories include only files and source codes to define the simulation scenario. 

## Development style
- Repository settings
  - Branch structure
    ```
    - main        # The latest operation guaranteed codes for general users
    - develop     # The latest buildable codes for S2E primary developers
    - feature/*   # Developing codes
    ```
  - We recommend that the general users use the `main` branch or suitable released version because we sometimes update the `develop` branch without backward compatibility, and users need to modify their user side codes and settings.
  - Push to `main` and `develop` is prohibited. All developers have to develop with `feature/*` branch and make a pull request.
  - Maintainers confirm the request and merge it to the `develop` branch.
  - [Reference document for the development style](https://nvie.com/posts/a-successful-git-branching-model/) 

- Flow of development
  1. Make a `feature/*` branch from the `develop` branch.
  2. Edit, commit, and push in the `feature/*` branch.
  3. Create a new pull request to the `develop` branch.
  4. A maintainer reviews the pull request. If some problems are found, the maintainer proposes modifications.
  5. According to the maintainer's proposal, the developer modifies the codes and goes back to 3.
  6. The maintainer merges the `feature/*` branch to the `develop` branch.
  7. The code owners decide to merge the `develop` branch to the `main` branch and release a new version.

- Binary files
  - Binary file commit is prohibited.
  - Please write the link to such files, or make a script file to get the files.
  - **Exception**
    - Images for markdown document files are allowable when the file size is smaller than 200K Bytes.

## Release style

- We use [Semantic Versioning 2.0.0](https://semver.org/) as versioning style
  - Basic version format is `<major>.<minor>.<patch>`(like `4.0.0`)
  - Public API is declared in the code itself(currently, there is no definitive list)
- All release should be tagged as `v<semver>`(like `v4.0.0`)
- These tags also should be release on [GitHub Releases](https://github.com/ut-issl/s2e-core/releases)
- Before S2E was released as OSS, a different versioning was adopted(missing patch version)
  - These versions are aliased with tags in the corresponding semver style(`v2.0` -> `v2.0.0`)

## Documents

- Documents for S2E are summarized in [s2e-documents](https://ut-issl.github.io/s2e-documents).

## How to use

- Please see [s2e-documents' getting started page](https://ut-issl.github.io/s2e-documents/Tutorials/GettingStarted.html).
