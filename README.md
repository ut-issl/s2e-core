# S2E CORE
[![check format](https://github.com/ut-issl/s2e-core/actions/workflows/check-format.yml/badge.svg)](https://github.com/ut-issl/s2e-core/actions/workflows/check-format.yml)
[![Build](https://github.com/ut-issl/s2e-core/actions/workflows/build.yml/badge.svg)](https://github.com/ut-issl/s2e-core/actions/workflows/build.yml)
[![GoogleTest](https://github.com/ut-issl/s2e-core/actions/workflows/google-test.yml/badge.svg)](https://github.com/ut-issl/s2e-core/actions/workflows/google-test.yml)

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
    - hotfix/*    # Bug Fix codes
    ```
  - We recommend that the general users use the `main` branch or suitable released version because we sometimes update the `develop` branch without backward compatibility, and users need to modify their user side codes and settings.
  - Push to `main` and `develop` is prohibited. All developers have to develop with `feature/*` or `hotfix/*` branch and make a pull request.
  - Maintainers confirm the request and merge it to the `develop` or `main` branch.
  - [Reference document for the development style](https://nvie.com/posts/a-successful-git-branching-model/) 

- Flow of development
  1. Make a `feature/*` branch from the `develop` branch.
     - To fix the small bugs in the latest release codes, please make `hotfix/*` branch from the `main` branch
  2. Edit, commit, and push in the branch.
     - Please check the [coding convention](https://github.com/ut-issl/s2e-documents/blob/develop/General/CodingConvention.md) and the `code format` in next section.
  3. Create a new pull request to the `develop` branch.
     - The target branch becomes the `main` branch for the `hotfix/*` branches.
  4. A maintainer reviews the pull request. If some problems are found, the maintainer proposes modifications.
  5. According to the maintainer's proposal, the developer modifies the codes and goes back to 3.
  6. The maintainer merges the `feature/*` branch to the `develop` branch.
  7. The code owners decide to merge the `develop` branch to the `main` branch and release a new version.

- Binary files
  - Binary file commit is prohibited.
  - Please write the link to such files, or make a script file to get the files.
  - **Exception**
    - Images for markdown document files are allowable when the file size is smaller than 200K Bytes.

- Code format
  - We use [clang-format](https://clang.llvm.org/docs/ClangFormat.html) for format source code.
  - We recommend install clang-format and format code before commit. It also will be checked on CI.
  - Some modern editor has plugin/extension for code format. It will be very useful.
    - VSCode: [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
    - Vim: [vim-clang-format](https://github.com/rhysd/vim-clang-format)

## Release style

- We use [Semantic Versioning 2.0.0](https://semver.org/) as versioning style
  - Basic version format is `<major>.<minor>.<patch>`(like `4.0.0`)
  - Public API is declared in the code itself(currently, there is no definitive list)
- All release should be tagged as `v<semver>`(like `v4.0.0`)
- These tags also should be release on [GitHub Releases](https://github.com/ut-issl/s2e-core/releases)
- Before S2E was released as OSS, a different versioning was adopted(missing patch version)
  - These versions are aliased with tags in the corresponding semver style(`v2.0` -> `v2.0.0`)

## Documents

- Documents for S2E are summarized in [s2e-documents](https://github.com/ut-issl/s2e-documents).

## How to use S2E

- Please see [s2e-documents' getting started page](https://github.com/ut-issl/s2e-documents/blob/develop/Tutorials/GettingStarted.md).
- We also have the `Development environment manuals`. You can find the link in the [Discussion page](https://github.com/ut-issl/s2e-core/discussions)
  - Sorry, but we only have manuals written in Japanese now. We need help to translate them.
- Support compiler
  - The following compilers are supported. The details are shown in GitHub Actions' results.
    - gcc/g++
    - clang++
    - Visual Studio C++

## Examples of User side repository

- [S2E-USER-EXAMPLE](https://github.com/ut-issl/s2e-user-example)
  - Sample codes for tutorials in the s2e-documents
- [S2E-FF](https://github.com/ut-issl/s2e-ff)
  - An example of S2E user side repository for Formation Flying study.
- [S2E-AOBC](https://github.com/ut-issl/s2e-aobc)
  - A user side repository for the AOCS module.

## Used Projects

| Project Name          | Developer                             | Launch  | Refs       |
| ----------------------| ------------------------------------  | ------- | ---------- |
| MAGNARO               | Nagoya Univ.                          | 2022    | -          |
| EQUULEUS              | ISSL, UT and JAXA                     | 2022    | -          |
| Optimal-1             | ArkEdge Space Inc.                    | 2023    | -          |
| SPHERE-1 EYE          | Sony Group Corporation and ISSL, UT   | 2023    | [Sekine 2023](https://archive.ists.ne.jp/upload_pdf/2023-f-6-02.pdf) |
| ONGLAISAT             | ISSL, UT                              | -       | [Ikari 2022](https://doi.org/10.57350/jesa.63) |


## Collaborators

[<img src="./data/img/arkedgespace_logo.png" width="25%" alt="ArkEdge Space Inc.">](https://arkedgespace.com/)


## Publications
1. S. Ikari, and et al., "Development of Compact and Highly Capable Integrated AOCS Module for CubeSats", Journal of Evolving Space Activities, vol. 1, ID 63, 2023. [Link](https://doi.org/10.57350/jesa.63)
1. 五十里, 他, "宇宙開発の効率化・高度化を目指した東京大学中須賀・船瀬研のOSS活動", UNISEC2022-04, 12th UNISEC Space Takumi Conference, 2022. [Link](http://unisec.jp/archives/7836)
1. H. Sekine, and et al., "Development of Software-In-the-Loop Simulator and Hardware-In-the-Loop Simulator of AOCS Module for CubeSats", 34th ISTS, Kurume, 2023. [Link](https://archive.ists.ne.jp/upload_pdf/2023-f-6-02.pdf)
