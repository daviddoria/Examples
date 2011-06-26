cmake_minimum_required(VERSION 2.6)
PROJECT(ImageEllipsoidSource)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageEllipsoidSource ImageEllipsoidSource.cxx)
TARGET_LINK_LIBRARIES(ImageEllipsoidSource vtkHybrid)

