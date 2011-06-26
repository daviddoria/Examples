cmake_minimum_required(VERSION 2.6)

PROJECT(UserEvent)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(UserEvent UserEvent.cxx vtkTestFilter.cxx)

TARGET_LINK_LIBRARIES(UserEvent vtkHybrid)
