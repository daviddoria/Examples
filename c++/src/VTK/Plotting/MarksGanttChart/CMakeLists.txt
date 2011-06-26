cmake_minimum_required(VERSION 2.6)

PROJECT(MarksGanttChart)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MarksGanttChart MarksGanttChart.cxx)

TARGET_LINK_LIBRARIES(MarksGanttChart vtkHybrid vtkCharts)
