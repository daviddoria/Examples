cmake_minimum_required(VERSION 2.6)
PROJECT(TextSource)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TextSource TextSource.cxx)
TARGET_LINK_LIBRARIES(TextSource vtkHybrid)

