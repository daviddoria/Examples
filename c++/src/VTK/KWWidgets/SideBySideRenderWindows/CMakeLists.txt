cmake_minimum_required(VERSION 2.6)
 
PROJECT(SideBySideRenderWindows)
 
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
 
find_package(KWWidgets REQUIRED)
include(${KWWidgets_USE_FILE})

ADD_EXECUTABLE(SideBySideRenderWindows SideBySideRenderWindows.cxx)
 TARGET_LINK_LIBRARIES(SideBySideRenderWindows vtkHybrid ${KWWidgets_LIBRARIES})
