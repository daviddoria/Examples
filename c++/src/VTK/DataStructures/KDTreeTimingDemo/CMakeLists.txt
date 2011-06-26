cmake_minimum_required(VERSION 2.6)

PROJECT(KDTreeTimingDemo)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(KDTreeTimingDemo KDTreeTimingDemo.cxx)
TARGET_LINK_LIBRARIES(KDTreeTimingDemo vtkHybrid vtkCharts)

