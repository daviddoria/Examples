cmake_minimum_required(VERSION 2.6)
PROJECT(ConnectivityFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ConnectivityFilter ConnectivityFilter.cxx)
TARGET_LINK_LIBRARIES(ConnectivityFilter vtkHybrid)

