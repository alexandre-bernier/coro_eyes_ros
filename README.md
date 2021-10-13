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
| Open3D (optional) | 0.13.0 |

## Installation

### Dependencies
    
- [CoRo Eyes SDK](https://github.com/alexandre-bernier/coro_eyes_sdk)<br />
    Follow instructions [<b>here</b>](https://github.com/alexandre-bernier/coro_eyes_sdk/blob/main/README.md).
    
- [Open3D](http://www.open3d.org/) (optional)

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


### Configurations

Before building, go to `coro_eyes_ros/config/dlp_plataforms/projector_settings.txt` and make sure that
`LCR4500_PARAMETERS_DLPC350_FIRMWARE` and `LCR4500_PARAMETERS_DLPC350_FLASH_PARAMETERS` have the right path:
`<your_catkin_ws>/src/coro_eyes_ros/config/dlp_platforms/LCr4500/[...]`.

### Building from source

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_ws/src
	git clone https://github.com/alexandre-bernier/coro_eyes_ros.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

## Usage

Multiple launch files are available depending on what you wish to do:

- <b>Upload patterns</b>

This should only be done once or if you wish to change the projected patterns.
You most likely won't ever have to do this since patterns should already have been uploaded.
    
        roslaunch coro_eyes_ros upload_patterns.launch

- <b>Adjust focus</b>

If the height of the CoRo Eyes has changed, you should make sure that the projector AND the cameras are still in focus.

        roslaunch coro_eyes_ros adjust_focus.launch

- <b>Camera calibration</b>

If the cameras moved on the CoRo Eyes, it's important to re-calibrate them. Run the script with the command below and
take pictures while presenting the calibration board in as many orientation and position as possible.
The more you cover each camera's sensor with the chessboard, the better the calibration. You also need to make sure
that the chessboard is always visible by both cameras, otherwise the picture you took will be discarded.

        roslaunch coro_eyes_ros camera_calibration.launch

- <b>Scan</b>

Once everything is set up, you can launch the scan node by calling the command below. This will initialize everything
and wait for a call of `coro_eyes/Scan` service to initiate a single scan. The resulting point cloud can be recovered
either in the service call response or by subscribing to the `/coro_eyes/point_cloud` topic.

Arguments:

- `show_camera_feed` You can show or hide the camera feed. Default: `false`.

        roslaunch coro_eyes_ros scan.launch

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/alexandre-bernier/coro_eyes_ros/issues).