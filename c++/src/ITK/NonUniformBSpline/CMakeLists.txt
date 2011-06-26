cmake_minimum_required(VERSION 2.6)

PROJECT(NonUniformBSpline)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(NonUniformBSpline NonUniformBSpline.cxx)
TARGET_LINK_LIBRARIES(NonUniformBSpline ${ITK_LIBRARIES})
