cmake_minimum_required(VERSION 2.6)

PROJECT(SCurveSpline)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SCurveSpline SCurveSpline.cxx)
TARGET_LINK_LIBRARIES(SCurveSpline vtkHybrid vtkInfovis)

