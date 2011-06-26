cmake_minimum_required(VERSION 2.6)
PROJECT(ShrinkWrapping)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ShrinkWrapping ShrinkWrapping.cxx)
TARGET_LINK_LIBRARIES(ShrinkWrapping vtkHybrid)

