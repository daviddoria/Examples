cmake_minimum_required(VERSION 2.6)

PROJECT(CovariantVectorNorm)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(CovariantVectorNorm CovariantVectorNorm.cxx)
TARGET_LINK_LIBRARIES(CovariantVectorNorm ${ITK_LIBRARIES})