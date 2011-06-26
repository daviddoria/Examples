cmake_minimum_required(VERSION 2.6)
PROJECT(SelectVisiblePoints)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SelectVisiblePoints SelectVisiblePoints.cxx)
TARGET_LINK_LIBRARIES(SelectVisiblePoints vtkHybrid)

