cmake_minimum_required(VERSION 2.6)
PROJECT(RandomNumbers)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(RandomNumbers RandomNumbers.cxx)
TARGET_LINK_LIBRARIES(RandomNumbers vtkHybrid)

