cmake_minimum_required(VERSION 2.6)

PROJECT(RenderViewBase)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(RenderViewBase RenderViewBase.cxx)
TARGET_LINK_LIBRARIES(RenderViewBase vtkHybrid vtkViews)

