cmake_minimum_required(VERSION 2.6)

project(Test)

# Tell CMake to also look in the source directory to find FindUnitTest++.cmake
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH})

#FIND_PACKAGE(UNITTEST++ REQUIRED)
find_package(UnitTest++ REQUIRED) # this seems to be case sensitive
include_directories(${UNITTEST++_INCLUDE_DIR})
link_directories(${UNITTEST++_LIBRARY})

add_executable(SingleTest SingleTest.cpp)
target_link_libraries(SingleTest UnitTest++)

add_executable(Suite Suite.cpp)
target_link_libraries(Suite UnitTest++)