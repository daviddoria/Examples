cmake_minimum_required(VERSION 2.6)
PROJECT(AnimationScene)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(AnimationScene AnimationScene.cxx)
TARGET_LINK_LIBRARIES(AnimationScene vtkHybrid)

