cmake_minimum_required(VERSION 2.6)

PROJECT(LandmarkBasedTransformInitializer)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(LandmarkBasedTransformInitializer LandmarkBasedTransformInitializer.cxx)
TARGET_LINK_LIBRARIES(LandmarkBasedTransformInitializer ${ITK_LIBRARIES})
