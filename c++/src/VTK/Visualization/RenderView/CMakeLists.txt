cmake_minimum_required(VERSION 2.6)

PROJECT(RenderView)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(RenderView RenderView.cxx)
TARGET_LINK_LIBRARIES(RenderView vtkHybrid vtkViews)

