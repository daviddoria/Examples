cmake_minimum_required(VERSION 2.6)

PROJECT(PieChart)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PieChart PieChart.cxx)

TARGET_LINK_LIBRARIES(PieChart vtkHybrid vtkCharts)
