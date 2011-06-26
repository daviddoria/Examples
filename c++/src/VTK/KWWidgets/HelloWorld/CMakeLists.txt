cmake_minimum_required(VERSION 2.6)
 
PROJECT(HelloWorld)
 
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
 
find_package(KWWidgets REQUIRED)
include(${KWWidgets_USE_FILE})

ADD_EXECUTABLE(HelloWorld HelloWorld.cxx)
 
TARGET_LINK_LIBRARIES(HelloWorld vtkHybrid ${KWWidgets_LIBRARIES})
