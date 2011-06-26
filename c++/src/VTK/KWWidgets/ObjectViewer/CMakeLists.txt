cmake_minimum_required(VERSION 2.6)
 
PROJECT(ObjectViewer)
 
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
 
find_package(KWWidgets REQUIRED)
include(${KWWidgets_USE_FILE})

ADD_EXECUTABLE(ObjectViewer ObjectViewer.cxx)
 
TARGET_LINK_LIBRARIES(ObjectViewer vtkHybrid ${KWWidgets_LIBRARIES})
