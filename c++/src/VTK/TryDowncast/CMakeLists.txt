cmake_minimum_required(VERSION 2.6)
PROJECT(TryDowncast)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TryDowncast TryDowncast.cxx)
TARGET_LINK_LIBRARIES(TryDowncast vtkHybrid)

