cmake_minimum_required(VERSION 2.6)

PROJECT(KochanekSpline)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(KochanekSpline KochanekSpline.cxx)
TARGET_LINK_LIBRARIES(KochanekSpline vtkHybrid vtkInfovis)

