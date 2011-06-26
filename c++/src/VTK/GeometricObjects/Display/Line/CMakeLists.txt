cmake_minimum_required(VERSION 2.6)

PROJECT(Line)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Line Line.cxx)
TARGET_LINK_LIBRARIES(Line vtkHybrid)

ADD_EXECUTABLE(DisplayColoredLines DisplayColoredLines.cxx)
TARGET_LINK_LIBRARIES(DisplayColoredLines vtkHybrid)

ADD_EXECUTABLE(LongLine LongLine.cxx)
TARGET_LINK_LIBRARIES(LongLine vtkHybrid)
