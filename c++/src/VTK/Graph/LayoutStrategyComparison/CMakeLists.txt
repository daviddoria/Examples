cmake_minimum_required(VERSION 2.6)

PROJECT(LayoutStrategyComparison)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(LayoutStrategyComparison LayoutStrategyComparison.cxx)
TARGET_LINK_LIBRARIES(LayoutStrategyComparison vtkHybrid vtkInfovis vtkViews)

