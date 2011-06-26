cmake_minimum_required(VERSION 2.6)
PROJECT(Follower)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Follower Follower.cxx)
TARGET_LINK_LIBRARIES(Follower vtkHybrid)

