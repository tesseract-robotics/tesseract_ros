^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2021-12-16)
------------------

0.3.0 (2021-12-06)
------------------

0.2.2 (2021-11-30)
------------------
* Add missing build depend to tesseract_msgs
* Contributors: Levi Armstrong

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
* Update Tesseract removed deprecated code
* Clean up environment monitor and interface
* Fix trail visualization and fix processing of empty commands message
* Update for changes with CollisionMarginData
* Add TaskInfo message
* Add optional Environment to EnvironmentState.msg
* Change TesseractState.msg to EnvironmentState.msg
* Updates to PlanningResponseArchive viewer
* Add replace link and joint support (`#85 <https://github.com/tesseract-robotics/tesseract_ros/issues/85>`_)
* Update to latest tesseract_environment changes and fix online planning example
* Update to leverage new visualizaton interface
* Move all packages out of tesseract_ros sub directory
* Contributors: Levi Armstrong, Levi-Armstrong, Matthew Powelson

0.1.0 (2020-12-02)
------------------
* WIP: Move ROS package into sub folder
* Switch to using built in Collision Shapes
* Merge pull request `#59 <https://github.com/tesseract-robotics/tesseract_ros/issues/59>`_ from arocchi/acm_fixes
  ACM  improvements: serialization and tests
* Fixed typo in documentation for AllowedCollisionEntry.msg
* Added AllowedCollisionEntry.msg in tesseract_msgs
* TesseractState includes information on allowed collisions, and ros_tesseract_utils are able to use them to correctly serialize and deserialize tesseract_ros::ROSBasicEnv from TesseractState messages
* Add additional compiler warning options
* Implement synchronous "compute_contact_reports" service in contact_monitor.cpp
* Fixed typo 'constacts' in ContactResultVector.msg
* Merge pull request `#26 <https://github.com/tesseract-robotics/tesseract_ros/issues/26>`_ from Levi-Armstrong/issue/FixContactMonitor
  Update contact monitor to use the latest version
* Fix the contact monitor to use the new contact managers
* Move tesseract into its own repository
* Contributors: Alessio Rocchi, John Wason, Levi, Levi Armstrong
