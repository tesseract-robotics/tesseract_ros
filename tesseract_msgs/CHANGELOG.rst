^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.27.0 (2024-12-01)
-------------------
* Updates to support visualizing online planning example
* Contributors: Levi Armstrong

0.26.0 (2024-10-27)
-------------------

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
* Update planning request input to leverage AnyPoly
* Contributors: Levi Armstrong

0.20.1 (2023-10-30)
-------------------

0.20.0 (2023-09-30)
-------------------

0.19.0 (2023-09-06)
-------------------

0.18.1 (2023-07-10)
-------------------

0.18.0 (2023-07-03)
-------------------
* Expose flag for generating DOT Graph in planning request
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
* Update planner server to leverage task composer plugins
* Fix toMsg and fromMsg for capsule (`#194 <https://github.com/tesseract-robotics/tesseract_ros/issues/194>`_)
* Contributors: Levi Armstrong, Matthew Powelson

0.14.0 (2022-10-23)
-------------------
* Update to use modify allowed collisions command
* Remove planning archive plugin
* Replace tesseract_process_managers with tesseract_task_composer package
* Contributors: Levi Armstrong

0.6.0 (2022-08-25)
------------------
* Add initial_state to the PlanningResponse message
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

0.4.2 (2022-04-25)
------------------

0.4.1 (2022-04-13)
------------------

0.4.0 (2022-04-08)
------------------

0.3.3 (2022-02-22)
------------------

0.3.2 (2022-01-21)
------------------
* Add save_io to planning request message (`#142 <https://github.com/tesseract-robotics/tesseract_ros/issues/142>`_)
  * Add save_io to planning request message
  * Update dependency version in rosinstall
* Contributors: Levi Armstrong

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
