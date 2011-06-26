cmake_minimum_required(VERSION 2.6)

PROJECT(PlaneSpatialObject)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(PlaneSpatialObject PlaneSpatialObject.cxx)
TARGET_LINK_LIBRARIES(PlaneSpatialObject ${ITK_LIBRARIES})
