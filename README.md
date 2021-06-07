# Tesseract ROS

Platform             | CI Status
---------------------|:---------
Linux (Focal)        | [![Build Status](https://github.com/ros-industrial-consortium/tesseract_ros/workflows/Focal-Build/badge.svg)](https://github.com/ros-industrial-consortium/tesseract_ros/actions)
Linux (Bionic)       | [![Build Status](https://github.com/ros-industrial-consortium/tesseract_ros/workflows/Bionic-Build/badge.svg)](https://github.com/ros-industrial-consortium/tesseract_ros/actions)
Linux (Unstable)     | [![Build Status](https://github.com/ros-industrial-consortium/tesseract_ros/workflows/Unstable-Build/badge.svg)](https://github.com/ros-industrial-consortium/tesseract_ros/actions)
Lint  (Clang-Format) | [![Build Status](https://github.com/ros-industrial-consortium/tesseract_ros/workflows/Clang-Format/badge.svg)](https://github.com/ros-industrial-consortium/tesseract_ros/actions)
Lint  (Clang Tidy)   | [![Build Status](https://github.com/ros-industrial-consortium/tesseract_ros/workflows/Clang-Tidy/badge.svg)](https://github.com/ros-industrial-consortium/tesseract_ros/actions)

[![Github Issues](https://img.shields.io/github/issues/ros-industrial-consortium/tesseract_ros.svg)](http://github.com/ros-industrial-consortium/tesseract_ros/issues)

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![license - bsd 2 clause](https://img.shields.io/:license-BSD%202--Clause-blue.svg)](https://opensource.org/licenses/BSD-2-Clause)

[![support level: consortium](https://img.shields.io/badge/support%20level-consortium-brightgreen.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

## Tesseract ROS Packages

* **tesseract_examples** – This package contains examples using tesseract and tesseract_ros for motion planning and collision checking.
* **tesseract_plugins** – This contains plugins for collision and kinematics which are automatically loaded by the monitors.
* **tesseract_rosutils** – This package contains the utilities like converting from ROS message types to native Tesseract types and the reverse.
* **tesseract_msgs** – This package contains the ROS message types used by Tesseract ROS.
* **tesseract_rviz** – This package contains the ROS visualization plugins for Rviz to visualize Tesseract. All of the features have been composed in libraries to enable to the ability to create custom displays quickly.
* **tesseract_monitoring** – This package contains different types of environment monitors. It currently contains a contact monitor and environment monitor. The contact monitor will monitor the active environment state and publish contact information. This is useful if the robot is being controlled outside of ROS, but you want to make sure it does not collide with objects in the environment. The second is the environment monitor, which is the main environment which facilitates requests to add, remove, disable and enable collision objects, while publishing its current state to keep other ROS nodes updated with the latest environment.
* **tesseract_planning_server** - This package contains a planning server supporting asynchronous execution of multiple planning requests.

## TODO's

.. Warning:: These packages are under heavy development and are subject to change.


See [issue #66](https://github.com/ros-industrial-consortium/tesseract/issues/66)

## Install Instructions

Clone this repository and the packages in the dependencies.rosinstall into your workspace. Build using [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/) or something similar

.. NOTE: To speed up clean build you may want to add tesseract_ext to an extended workspace.

## Tesseract Examples
### Online Planning Example
This example demonstrates using TrajOpt to plan in an "online" manner. Use the joint state publisher gui to change the location of the collision obstacle or the target and watch it dynamically plan. Adjust the box_size parameter for faster adaption.

```roslaunch tesseract_ros_examples online_planning_example.launch```

![Online Planning Example](gh_pages/_static/examples/online_planning_example.gif)

## Building with Clang-Tidy Enabled

Must pass the -DTESSERACT_ENABLE_CLANG_TIDY=ON to cmake when building. This is automatically enabled if cmake argument -DTESSERACT_ENABLE_TESTING_ALL=ON is passed.

## Running Tesseract ROS Tests

Tesseract ROS Packages use the typical method so pass catkin argument `run_tests` when wanting to run these tests.

## Build Branch Sphinx Documentation

```
cd gh_pages
sphinx-build . output
```
Now open gh_pages/output/index.rst and remove *output* directory before commiting changes.
