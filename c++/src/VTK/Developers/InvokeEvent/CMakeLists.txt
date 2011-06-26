cmake_minimum_required(VERSION 2.6)

PROJECT(InvokeEvent)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(InvokeEvent InvokeEvent.cxx vtkTestFilter.cxx vtkDerivedFilter.cxx)
TARGET_LINK_LIBRARIES(InvokeEvent vtkHybrid)
