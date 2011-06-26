cmake_minimum_required(VERSION 2.6)

Project(FileSystem)

FIND_PACKAGE(Boost 1.38 COMPONENTS file_system required)

INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${LINK_DIRECTORIES} ${Boost_LIBRARY_DIRS})

ADD_EXECUTABLE(FileSystem FileSystem.cpp)
target_link_libraries(FileSystem boost_filesystem boost_system)

