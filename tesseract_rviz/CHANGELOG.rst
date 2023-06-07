^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_rviz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.17.0 (2023-06-07)
-------------------
* Update based on changes to tesseract_qt to support studio plugins (`#212 <https://github.com/tesseract-robotics/tesseract_ros/issues/212>`_)
* Contributors: Levi Armstrong

0.16.2 (2023-04-28)
-------------------

0.16.1 (2023-04-11)
-------------------
* Add environment snapshot viewing to environment monitor
* Contributors: Levi Armstrong

0.16.0 (2023-04-10)
-------------------

0.15.2 (2023-03-14)
-------------------
* Fix tesseract rviz export libraries
* Contributors: Levi Armstrong

0.15.1 (2023-03-05)
-------------------

0.15.0 (2023-03-04)
-------------------
* Update to leverage tesseract_qt event filters (`#199 <https://github.com/tesseract-robotics/tesseract_ros/issues/199>`_)
* Contributors: Levi Armstrong

0.14.0 (2022-10-23)
-------------------
* Update to use modify allowed collisions command
* Remove planning archive plugin
* Remove legacy rviz plugins
* Contributors: Levi Armstrong

0.6.0 (2022-08-25)
------------------
* Fix ogre getAABB
* Disable publishing tf in environment monitor properties
* Update to use new Poly types in tesseract_planning
* Update to latest tesseract
* Contributors: Levi Armstrong

0.5.1 (2022-06-21)
------------------
* Add cartesian interactive marker support
* Add joint interactive marker to manipulation widget
* Contributors: Levi Armstrong

0.5.0 (2022-05-17)
------------------
* Update to use tesseract_qt (`#154 <https://github.com/tesseract-robotics/tesseract_ros/issues/154>`_)
* Contributors: Levi Armstrong

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
* Contributors: Levi Armstrong

0.3.0 (2021-12-06)
------------------
* Support moving AllowedCollisionMatrix into tesseract_common namespace
* Contributors: Matthew Powelson

0.2.2 (2021-11-30)
------------------

0.2.1 (2021-11-30)
------------------
* Cleanup CMakeLists.txt
* Contributors: Levi Armstrong

0.2.0 (2021-11-04)
------------------
* Update due to changes with contact manager plugins
* Improve manipulation widget support for external positioners
* Fix manipulator widget updating config segfault
* Update to Joint and Kinematic group (`#125 <https://github.com/tesseract-robotics/tesseract_ros/issues/125>`_)
* Remove References to Deprecated Tesseract_geometry Functions (`#124 <https://github.com/tesseract-robotics/tesseract_ros/issues/124>`_)
* Update Tesseract removed deprecated code
* Clean up environment monitor and interface
* Add online example rviz config and fix trajectory display after disable
* Update due to switching to boost serialization
* Remove use of isWithinLimits and use satisfiesPositionLimits
* Fix trail visualization and fix processing of empty commands message
* Clang format
* Check for empty xml in PlanningRequestArchiveViewer
* Add optional Environment to EnvironmentState.msg
* Change TesseractState.msg to EnvironmentState.msg
* Refactor RVIZ trajectory widget to allow it to be reused
* Updates to PlanningResponseArchive viewer
* Update to new forward and inverse kinematics interface
* Updates to support fromXML templates
* Add replace link and joint support (`#85 <https://github.com/tesseract-robotics/tesseract_ros/issues/85>`_)
* Update to latest tesseract_environment changes and fix online planning example
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Update to leverage new visualizaton interface
* Move all packages out of tesseract_ros sub directory
* Contributors: DavidMerzJr, Levi Armstrong, Levi-Armstrong, Matthew Powelson

0.1.0 (2020-12-02)
------------------
* WIP: Move ROS package into sub folder
* Isolate tesseract_collision namespace
* Switch to using built in Collision Shapes
* Fix formatting using clang
* Fix warnings in unit tests
* Add additional compiler warning options
* Updated bullet_ros to not build unit tests; added line for installation of plugin XML files in Rviz package
* Eigen Alignment fixes
* Add monitoring of joint state topic to tesseract state display
* Merge pull request `#20 <https://github.com/tesseract-robotics/tesseract_ros/issues/20>`_ from Levi-Armstrong/feature/Isometry3d
  switch from using affine3d to isometry3d
* switch from using affine3d to isometry3d
* Move tesseract into its own repository
* Contributors: Levi, Levi Armstrong, Matthew Powelson, mripperger
