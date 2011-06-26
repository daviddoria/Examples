cmake_minimum_required(VERSION 2.6)

PROJECT(TreeMapView)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(TreeMapView TreeMapView.cxx)
TARGET_LINK_LIBRARIES(TreeMapView vtkHybrid vtkInfovis vtkViews)

