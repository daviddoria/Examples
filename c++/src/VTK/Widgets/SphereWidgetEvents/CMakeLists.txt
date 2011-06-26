cmake_minimum_required(VERSION 2.6)

PROJECT(SphereWidgetEvents)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SphereWidgetEvents SphereWidgetEvents.cxx)
TARGET_LINK_LIBRARIES(SphereWidgetEvents vtkHybrid vtkWidgets)
