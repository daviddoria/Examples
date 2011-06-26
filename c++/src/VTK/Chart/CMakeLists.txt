cmake_minimum_required(VERSION 2.6)

PROJECT(Chart)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Chart Chart.cxx)
TARGET_LINK_LIBRARIES(Chart vtkHybrid vtkCharts)

ADD_EXECUTABLE(MemoryLeak MemoryLeak.cxx)
TARGET_LINK_LIBRARIES(MemoryLeak vtkHybrid vtkCharts)
