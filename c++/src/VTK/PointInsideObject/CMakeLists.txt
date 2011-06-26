cmake_minimum_required(VERSION 2.6)
PROJECT(PointInsideObject)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PointInsideObject PointInsideObject.cxx)
TARGET_LINK_LIBRARIES(PointInsideObject vtkHybrid)

ADD_EXECUTABLE(EnclosedPoint EnclosedPoint.cxx)
TARGET_LINK_LIBRARIES(EnclosedPoint vtkHybrid)

