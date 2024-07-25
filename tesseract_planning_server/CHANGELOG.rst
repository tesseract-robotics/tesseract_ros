^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_planning_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.23.0 (2024-07-25)
-------------------
* Add environment as const to data storage
* Fix use-after-move (`#240 <https://github.com/tesseract-robotics/tesseract_ros/issues/240>`_)
* Fix output key name
* Update because TaskComposerProblem was removed
* Contributors: Levi Armstrong, Roelof Oomen

0.22.0 (2024-06-11)
-------------------
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
* Upgrade to tesseract version 0.20.0
* Update to leverage TaskComposerContext
* Contributors: Levi Armstrong

0.19.0 (2023-09-06)
-------------------
* Update to use tesseract package components
* Contributors: Levi Armstrong

0.18.1 (2023-07-10)
-------------------

0.18.0 (2023-07-03)
-------------------
* Expose flag for generating DOT Graph in planning request
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

0.15.2 (2023-03-14)
-------------------

0.15.1 (2023-03-05)
-------------------

0.15.0 (2023-03-04)
-------------------
* Update to leverage tesseract_qt event filters (`#199 <https://github.com/tesseract-robotics/tesseract_ros/issues/199>`_)
* Update planner server to leverage task composer plugins
* Contributors: Levi Armstrong

0.14.0 (2022-10-23)
-------------------
* Remove planning archive plugin
* Replace tesseract_process_managers with tesseract_task_composer package
* Update planning server to legacy simple planner
* Update to leverage legacy simple profile
* Contributors: Levi Armstrong

0.6.0 (2022-08-25)
------------------
* Add initial_state to the PlanningResponse message
* Update to use new Poly types in tesseract_planning
* Contributors: Levi Armstrong

0.5.1 (2022-06-21)
------------------

0.5.0 (2022-05-17)
------------------

0.4.4 (2022-05-13)
------------------

0.4.3 (2022-05-03)
------------------
* Update changes with serialization (`#151 <https://github.com/tesseract-robotics/tesseract_ros/issues/151>`_)
  * Update changes with serialization
  * Update rosinstall files
* Contributors: Levi Armstrong

0.4.2 (2022-04-25)
------------------
* Update to changes in ProcessPlanningFuture (`#150 <https://github.com/tesseract-robotics/tesseract_ros/issues/150>`_)
* Contributors: Levi Armstrong

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

0.2.1 (2021-11-30)
------------------
* Update to leverage namespaces in profile dictionary
* Cleanup CMakeLists.txt
* Contributors: Levi Armstrong

0.2.0 (2021-11-04)
------------------
* Update due to changes with contact manager plugins
* Update to Joint and Kinematic group (`#125 <https://github.com/tesseract-robotics/tesseract_ros/issues/125>`_)
* Clean up environment monitor and interface
* Update new tesseract_srdf package
* Update due to switching to boost serialization
* Clang format
* Check for empty xml in PlanningRequestArchiveViewer
* Change TesseractState.msg to EnvironmentState.msg
* Updates to PlanningResponseArchive viewer
* Updates to support fromXML templates
* Update to latest tesseract_environment changes and fix online planning example
* Expose ability to set planning servers number of threads
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Update simple planner profiles names
* Move all packages out of tesseract_ros sub directory
* Contributors: Levi Armstrong, Levi-Armstrong, Matthew Powelson

0.1.0 (2020-12-02)
------------------
