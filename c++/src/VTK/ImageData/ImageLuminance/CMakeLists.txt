cmake_minimum_required(VERSION 2.6)
PROJECT(ImageLuminance)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageLuminance ImageLuminance.cxx)
TARGET_LINK_LIBRARIES(ImageLuminance vtkHybrid)
