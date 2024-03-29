# Copyright 2022 by Daniel Winkelman. All rights reserved.

cmake_minimum_required (VERSION 3.16)

set (CMAKE_BUILD_TYPE Release)
set (CMAKE_CXX_STANDARD 20)

project (Algo CXX)
set (Target_lib algo)
set (Target_tests algo-tests)


if (NOT DEFINED ${DISABLE_TESTS})
    include (FetchContent)
    FetchContent_Declare (googletest URL https://github.com/google/googletest/archive/refs/tags/release-1.11.0.zip)
    FetchContent_MakeAvailable (googletest)
    enable_testing ()
endif ()

include_directories (src)
if (NOT DEFINED ${DISABLE_TESTS})
    include_directories (${googletest_SOURCE_DIR}/googletest/include)
endif ()

file (GLOB LIB_SOURCES src/algo/*.cc)
add_library (${Target_lib} ${LIB_SOURCES})

if (NOT DEFINED ${DISABLE_TESTS})
    file (GLOB TESTS_SOURCES tests/*.cc)
    add_executable (${Target_tests} ${TESTS_SOURCES})
    target_link_libraries (${Target_tests} ${Target_lib} gtest_main)

    include (GoogleTest)
    gtest_discover_tests (${Target_tests})
else ()
    message ("No Tests")
endif ()
