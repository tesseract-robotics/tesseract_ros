^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_rosutils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.27.0 (2024-12-01)
-------------------
* Updates to support visualizing online planning example
* Contributors: Levi Armstrong

0.26.0 (2024-10-27)
-------------------
* Update to latest tesseract
* Contributors: Levi Armstrong

0.25.0 (2024-09-29)
-------------------
* Update to support Geometry Type CompoundMesh
* Contributors: Levi Armstrong

0.24.0 (2024-08-15)
-------------------

0.23.0 (2024-07-25)
-------------------
* Update because TaskComposerProblem was removed
* Contributors: Levi Armstrong

0.22.0 (2024-06-11)
-------------------
* Update to change in task composer node info
* Leverage forward declarations (`#233 <https://github.com/tesseract-robotics/tesseract_ros/issues/233>`_)
* Contributors: Levi Armstrong

0.21.0 (2023-11-10)
-------------------

0.20.1 (2023-10-30)
-------------------

0.20.0 (2023-09-30)
-------------------

0.19.0 (2023-09-06)
-------------------
* Update to use tesseract package components
* Contributors: Levi Armstrong

0.18.1 (2023-07-10)
-------------------

0.18.0 (2023-07-03)
-------------------
* Changes to support task composer restructure
* Contributors: Levi Armstrong

0.17.0 (2023-06-07)
-------------------

0.16.2 (2023-04-28)
-------------------

0.16.1 (2023-04-11)
-------------------

0.16.0 (2023-04-10)
-------------------
* Add support for AddTrajectoryLinkCommand
* Contributors: Levi Armstrong

0.15.2 (2023-03-14)
-------------------

0.15.1 (2023-03-05)
-------------------

0.15.0 (2023-03-04)
-------------------
* Fix toMsg and fromMsg for capsule (`#194 <https://github.com/tesseract-robotics/tesseract_ros/issues/194>`_)
* Contributors: Matthew Powelson

0.14.0 (2022-10-23)
-------------------
* Update to use modify allowed collisions command
* Remove planning archive plugin
* Replace tesseract_process_managers with tesseract_task_composer package
* Fix not returning value for message conversion functions
* Add ros conversions for joint map
* Contributors: Levi Armstrong

0.6.0 (2022-08-25)
------------------
* Update to use new Poly types in tesseract_planning
* Contributors: Levi Armstrong

0.5.1 (2022-06-21)
------------------

0.5.0 (2022-05-17)
------------------

0.4.4 (2022-05-13)
------------------
* Add new RViz plugins using Tesseract widgets (`#152 <https://github.com/tesseract-robotics/tesseract_ros/issues/152>`_)
* Contributors: Levi Armstrong

0.4.3 (2022-05-03)
------------------
* Update changes with serialization (`#151 <https://github.com/tesseract-robotics/tesseract_ros/issues/151>`_)
  * Update changes with serialization
  * Update rosinstall files
* Contributors: Levi Armstrong

0.4.2 (2022-04-25)
------------------

0.4.1 (2022-04-13)
------------------

0.4.0 (2022-04-08)
------------------
* Update to use monitor interface and clean up environment monitor
* Contributors: Levi Armstrong

0.3.3 (2022-02-22)
------------------

0.3.2 (2022-01-21)
------------------

0.3.1 (2021-12-16)
------------------
* Fix bug in how geometry octree are converted from message and visualized
* Add missing visualization_msgs to tesseract_rosutils CMakelists.txt
* Contributors: Levi Armstrong

0.3.0 (2021-12-06)
------------------
* Update renaming of ContactManagerConfig variables
* Support moving AllowedCollisionMatrix into tesseract_common namespace
* Contributors: Levi Armstrong, Matthew Powelson

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
