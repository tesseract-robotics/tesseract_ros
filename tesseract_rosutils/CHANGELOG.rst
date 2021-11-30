^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_rosutils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.2 (2021-11-30)
------------------

0.2.1 (2021-11-30)
------------------
* Add contact margin data override MODIFY (`#133 <https://github.com/tesseract-robotics/tesseract_ros/issues/133>`_)
  * Add contact margin data override MODIFY
  * Update rosinstall tesseract hash
* Cleanup CMakeLists.txt
* Contributors: Levi Armstrong

0.2.0 (2021-11-04)
------------------
* Update due to changes with contact manager plugins
* Update to Joint and Kinematic group (`#125 <https://github.com/tesseract-robotics/tesseract_ros/issues/125>`_)
* Remove References to Deprecated Tesseract_geometry Functions (`#124 <https://github.com/tesseract-robotics/tesseract_ros/issues/124>`_)
* Update online planner to latest changes in trajopt ifopt package (`#119 <https://github.com/tesseract-robotics/tesseract_ros/issues/119>`_)
  Co-authored-by: ben-greenberg <benrgreenberg@gmail.com>
  Co-authored-by: ben <ben.greenberg@swri.org>
* Update Tesseract removed deprecated code
* Clean up environment monitor and interface
* Update new tesseract_srdf package
* Update due to switching to boost serialization
* Fix trail visualization and fix processing of empty commands message
* Update for changes with CollisionMarginData
* Clang format
* Add TaskInfo message
* Include joint state in to/from msg utils for Environment
* Add optional Environment to EnvironmentState.msg
* Change TesseractState.msg to EnvironmentState.msg
* Switch plotting of toolpath to use marker array to support namespaces
* Add replace link and joint support (`#85 <https://github.com/tesseract-robotics/tesseract_ros/issues/85>`_)
* Update to latest tesseract_environment changes and fix online planning example
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Update to leverage new visualizaton interface
* Move all packages out of tesseract_ros sub directory
* Contributors: DavidMerzJr, Levi Armstrong, Levi-Armstrong, Matthew Powelson

0.1.0 (2020-12-02)
------------------
