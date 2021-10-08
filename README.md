# coro_eyes_ros

## Overview

ROS package for the CoRo Eyes structured light camera. See [coro_eyes_sdk](https://github.com/alexandre-bernier/coro_eyes_sdk).

### License

The source code is released under a [BSD 3-Clause license](coro_eyes_sdk/LICENSE).

<b>Author: Alexandre Bernier<br />
Affiliation: [CoRo, ÉTS](http://en.etsmtl.ca/unites-de-recherche/coro/accueil?lang=en-CA)<br />
Maintainer: Alexandre Bernier, ab.alexandre.bernier@gmail.com</b>

### Compatibility

The coro_eyes_ros package has been tested on Ubuntu 20.04 with the following dependency versions:

| Dependencies | Version |
| --- | --- |
| coro_eyes_sdk | 1.0 |
| OPen3D | 0.13.0 |

## Installation

### Dependencies
    
- [CoRo Eyes SDK](https://github.com/alexandre-bernier/coro_eyes_sdk)<br />
    Follow instructions [<b>here</b>](https://github.com/alexandre-bernier/coro_eyes_sdk/blob/main/README.md).
    
- [Open3D](http://www.open3d.org/)

    1. Make sure you have CMake 3.18+ (<b>`cmake --version`</b>) and upgrade if necessary following these [instructions](https://apt.kitware.com/).

    2. Clone the repository in the location of your choice:
    
           git clone --recursive --depth 1 --branch v0.13.0 https://github.com/isl-org/Open3D.git
           
    3. Update the submodules:
    
           cd Open3D/
           git submodule update --init --recursive
           
    4. Install the dependencies:
    
           util/install_deps_ubuntu.sh
           
    5. Build and install the library (optional - you can change the PYTHON_EXECUTABLE variable for a virtual environment):
    
           mkdir build
           cd build/
           cmake -DBUILD_SHARED_LIBS=ON -DGLIBCXX_USE_CXX11_ABI=1 -DPYTHON_EXECUTABLE=/usr/bin/python3 ..
           sudo cmake --build . --config Release --parallel 12 --target install
           

### Building from source


