# coro_eyes_ros

## Overview

ROS package for the CoRo Eyes structured light camera. See [coro_eyes_sdk](https://github.com/alexandre-bernier/coro_eyes_sdk).

### License

The source code is released under a [BSD 3-Clause license](coro_eyes_sdk/LICENSE).

<b>Author: Alexandre Bernier <br />
Affiliation: [CoRo, ÉTS](http://en.etsmtl.ca/unites-de-recherche/coro/accueil?lang=en-CA) <br />
Maintainer: Alexandre Bernier, ab.alexandre.bernier@gmail.com</b>

### Compatibility

The coro_eyes_ros package has been tested on Ubuntu 20.04 with the following dependency versions:

| Dependencies | Version |
| --- | --- |
| coro_eyes_sdk | main |
| Open3D (optional) | 0.13.0 |

## Installation

### Dependencies
    
- [CoRo Eyes SDK](https://github.com/alexandre-bernier/coro_eyes_sdk) <br />
    Follow instructions [<b>here</b>](https://github.com/alexandre-bernier/coro_eyes_sdk/blob/main/README.md).
    
- [Open3D](http://www.open3d.org/) (optional)

      pip3 install open3d

### Configurations

Before building, go to `coro_eyes_ros/config/dlp_plataforms/projector_settings.txt` and make sure that
`LCR4500_PARAMETERS_DLPC350_FIRMWARE` and `LCR4500_PARAMETERS_DLPC350_FLASH_PARAMETERS` have the right path:
`<your_catkin_ws>/src/coro_eyes_ros/config/dlp_platforms/LCr4500/[...]`.

### Building from source

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_ws/src
	git clone https://github.com/alexandre-bernier/coro_eyes_ros.git
	cd ../
	catkin_make

## Usage

Multiple launch files are available depending on what you wish to do:

- <b>CoRo Eyes</b>

Launching this node will call the Scan node, the post-processing node `scripts/coro_eyes.py` and the point cloud
visualizer node `scripts/pc_visualizer.py`. The post-processing node listens to user inputs and calls the `coro_eyes/Scan` service. If you wish
to apply any post-processing with Open3D, you can write your own code in the `post_processing` function in
`scripts/coro_eyes.py`. The processed point cloud will be published to the `/coro_eyes/point_cloud` topic and displayed
by the visualizer node.

    roslaunch coro_eyes_ros coro_eyes.launch

Arguments:

`show_camera_feed` You can show or hide the camera feed. Default: `false`.

`show_visualizer` You can show or hide the point cloud visualizer. Default: `true`.

- <b>Scan</b>

Once everything is set up, you can launch the scan node by calling the command below. This will initialize everything
and wait for a call of `coro_eyes/Scan` service to initiate a single scan. The resulting point cloud can be recovered
either in the service call response or by subscribing to the `/coro_eyes/raw_point_cloud` topic.

    roslaunch coro_eyes_ros scan.launch

Arguments:

`show_camera_feed` You can show or hide the camera feed. Default: `false`.

- <b>Point cloud visualizer</b>

This node connects to a topic of type `sensor_msgs/PointCloud2` and displays the point cloud received.

    rosrun coro_eyes_ros pc_visualizer.py /coro_eyes/point_cloud

- <b>Camera calibration</b>

If the cameras moved on the CoRo Eyes, it's important to re-calibrate them. Run the script with the command below and
take pictures while presenting the calibration board in as many orientation and position as possible.
The more you cover each camera's sensor with the chessboard, the better the calibration. You also need to make sure
that the chessboard is always visible by both cameras, otherwise the picture you took will be discarded.

    roslaunch coro_eyes_ros camera_calibration.launch

- <b>Adjust focus</b>

If the height of the CoRo Eyes has changed, you should make sure that the projector and the cameras are still in focus.

    roslaunch coro_eyes_ros adjust_focus.launch

- <b>Upload patterns</b>

This should only be done once or if you wish to change the projected patterns.
You most likely won't ever have to do this since patterns should already have been uploaded.
    
    roslaunch coro_eyes_ros upload_patterns.launch
        
## URDF

A URDF is included in this package that puts a box around the physical CoRo Eyes hardware (including the protective acrylic sheet). The xacro file you need to include in your project is `urdf/coro_eyes.urdf.xacro`. You can find an example on how to inlucde the urdf in `urdf/coro_eyes_example.urdf.xacro` and `launch/load_urdf.launch`. The origin of the `coro_eyes_joint` should have the result of the `Pose estimation` node with the child being the user-defined reference frame used to compute the pose of the CoRo Eyes left camera.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/alexandre-bernier/coro_eyes_ros/issues).
