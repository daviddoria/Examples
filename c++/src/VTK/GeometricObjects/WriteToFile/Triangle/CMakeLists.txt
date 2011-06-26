cmake_minimum_required(VERSION 2.6)

PROJECT(Triangle)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Triangle Triangle.cxx)
TARGET_LINK_LIBRARIES(Triangle vtkHybrid)

ADD_EXECUTABLE(Triangle_indexed Triangle_indexed.cxx)
TARGET_LINK_LIBRARIES(Triangle_indexed vtkHybrid)

ADD_EXECUTABLE(Triangle_bad Triangle_bad.cxx)
TARGET_LINK_LIBRARIES(Triangle_bad vtkHybrid)
