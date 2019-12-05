#
# @file tesseract_macros.cmae
# @brief Common Tesseract CMake Macros
#
# @author Levi Armstrong
# @date October 15, 2019
# @version TODO
# @bug No known bugs
#
# @copyright Copyright (c) 2019, Southwest Research Institute
#
# @par License
# Software License Agreement (Apache License)
# @par
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# @par
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# This macro will add tesseract standard compiler option to a target
# Usage: tesseract_target_compile_options(target <INTERFACE|PUBLIC|PRIVATE>)
#    * c++14
#    * Warning (-Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
#    * disable avx due to eigen issues
#    * Add Clang Tidy
macro(tesseract_target_compile_options target)
  cmake_parse_arguments(ARG "INTERFACE;PUBLIC;PRIVATE" "" "" ${ARGN})

  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "tesseract_target_compile_options() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_14 CXX_FEATURE_FOUND)
  set(warning_flags -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)

  if (ARG_INTERFACE)
    target_compile_options("${target}" INTERFACE ${warning_flags})

    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      if(CXX_FEATURE_FOUND EQUAL "-1")
        target_compile_options("${target}" INTERFACE -std=c++14 -mno-avx)
      else()
        target_compile_features("${target}" INTERFACE cxx_std_14)
        target_compile_options("${target}" INTERFACE -mno-avx)
      endif()
    else()
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    endif()
  elseif(ARG_PUBLIC)
    target_compile_options("${target}" PRIVATE ${warning_flags})

    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      if(CXX_FEATURE_FOUND EQUAL "-1")
        target_compile_options("${target}" PUBLIC -std=c++14 -mno-avx)
      else()
        target_compile_features("${target}" PUBLIC cxx_std_14)
        target_compile_options("${target}" PUBLIC -mno-avx)
      endif()
    else()
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    endif()
  elseif(ARG_PRIVATE)
    target_compile_options("${target}" PRIVATE ${warning_flags})

    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      if(CXX_FEATURE_FOUND EQUAL "-1")
        target_compile_options("${target}" PRIVATE -std=c++14 -mno-avx)
      else()
        target_compile_features("${target}" PRIVATE cxx_std_14)
        target_compile_options("${target}" PRIVATE -mno-avx)
      endif()
    else()
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    endif()
  endif()
endmacro()

# Add clang-tidy to a target if ENABLE_CLANG_TIDY or ENABLE_TESTS is enabled
# Usage: tesseract_clang_tidy(Target) or tesseract_clang_tidy(Target true) or tesseract_clang_tidy(Target false)
#    * tesseract_clang_tidy(Target) adds clang tidy with warnings as errors
#    * tesseract_clang_tidy(Target true) adds clang tidy with warnings as errors
#    * tesseract_clang_tidy(Target false) adds clang tidy with warnings as warnings
macro(tesseract_clang_tidy target)
  cmake_parse_arguments(ARG "true,false" "" "" ${ARGN})

  get_target_property(${target}_type ${target} TYPE)

  # Add clang tidy
  if (NOT ${${target}_type} STREQUAL "INTERFACE_LIBRARY")
    if (ENABLE_CLANG_TIDY OR ENABLE_TESTS)
      find_program(CLANG_TIDY_EXE NAMES "clang-tidy" DOC "Path to clang-tidy executable")
      if(NOT CLANG_TIDY_EXE)
        message(WARNING "clang-tidy not found.")
      else()
        message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
        if(ARG_false)
          set(DO_CLANG_TIDY "${CLANG_TIDY_EXE}" "-checks=-*,bugprone-*,cppcoreguidelines-avoid-goto,cppcoreguidelines-c-copy-assignment-signature,cppcoreguidelines-interfaces-global-init,cppcoreguidelines-narrowing-conversions,cppcoreguidelines-no-malloc,cppcoreguidelines-slicing,cppcoreguidelines-special-member-functions,misc-*,modernize-*,performance-*,readability-avoid-const-params-in-decls,readability-container-size-empty,readability-delete-null-pointer,readability-deleted-default,readability-else-after-return,readability-function-size,readability-identifier-naming,readability-inconsistent-declaration-parameter-name,readability-misleading-indentation,readability-misplaced-array-index,readability-non-const-parameter,readability-redundant-*,readability-simplify-*,readability-static-*,readability-string-compare,readability-uniqueptr-delete-release,readability-rary-objects")
        else()
          set(DO_CLANG_TIDY "${CLANG_TIDY_EXE}" "-checks=-*,bugprone-*,cppcoreguidelines-avoid-goto,cppcoreguidelines-c-copy-assignment-signature,cppcoreguidelines-interfaces-global-init,cppcoreguidelines-narrowing-conversions,cppcoreguidelines-no-malloc,cppcoreguidelines-slicing,cppcoreguidelines-special-member-functions,misc-*,modernize-*,performance-*,readability-avoid-const-params-in-decls,readability-container-size-empty,readability-delete-null-pointer,readability-deleted-default,readability-else-after-return,readability-function-size,readability-identifier-naming,readability-inconsistent-declaration-parameter-name,readability-misleading-indentation,readability-misplaced-array-index,readability-non-const-parameter,readability-redundant-*,readability-simplify-*,readability-static-*,readability-string-compare,readability-uniqueptr-delete-release,readability-rary-objects" "-warnings-as-errors=-*,bugprone-*,cppcoreguidelines-avoid-goto,cppcoreguidelines-c-copy-assignment-signature,cppcoreguidelines-interfaces-global-init,cppcoreguidelines-narrowing-conversions,cppcoreguidelines-no-malloc,cppcoreguidelines-slicing,cppcoreguidelines-special-member-functions,misc-*,modernize-*,performance-*,readability-avoid-const-params-in-decls,readability-container-size-empty,readability-delete-null-pointer,readability-deleted-default,readability-else-after-return,readability-function-size,readability-identifier-naming,readability-inconsistent-declaration-parameter-name,readability-misleading-indentation,readability-misplaced-array-index,readability-non-const-parameter,readability-redundant-*,readability-simplify-*,readability-static-*,readability-string-compare,readability-uniqueptr-delete-release,readability-rary-objects")
        endif()
        set_target_properties("${target}" PROPERTIES CXX_CLANG_TIDY "${DO_CLANG_TIDY}")
      endif()
    endif()
  endif()
endmacro()

# Performs multiple operation so other packages may find a package
# Usage: tesseract_configure_package(targetA targetb)
#   * It installs the provided targets
#   * It exports the provided targets under the namespace tesseract::
#   * It installs the package.xml file
#   * It create and install the ${PROJECT_NAME}-config.cmake and ${PROJECT_NAME}-config-version.cmake
macro(tesseract_configure_package)
  install(TARGETS ${ARGV}
          EXPORT ${PROJECT_NAME}-targets
          RUNTIME DESTINATION bin
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib)
  install(EXPORT ${PROJECT_NAME}-targets NAMESPACE tesseract:: DESTINATION lib/cmake/${PROJECT_NAME})

  install(FILES package.xml DESTINATION share/${PROJECT_NAME})

  # Create cmake config files
  include(CMakePackageConfigHelpers)
  configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

  write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME})

  export(EXPORT ${PROJECT_NAME}-targets NAMESPACE tesseract:: FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)
endmacro()

# This macro call the appropriate gtest function to add a test based on the cmake version
# Usage: tesseract_gtest_discover_tests(target)
macro(tesseract_gtest_discover_tests target)
  if(${CMAKE_VERSION} VERSION_LESS "3.10.0")
    gtest_add_tests(${target} "" AUTO)
  else()
    gtest_discover_tests(${target})
  endif()
endmacro()