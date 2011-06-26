cmake_minimum_required(VERSION 2.6)

PROJECT(Quality)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Quality Quality.cxx)
TARGET_LINK_LIBRARIES(Quality vtkHybrid)

