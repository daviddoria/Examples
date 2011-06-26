cmake_minimum_required(VERSION 2.6)

PROJECT(BinaryBallStructuringElement)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(BinaryBallStructuringElement BinaryBallStructuringElement.cxx)
TARGET_LINK_LIBRARIES(BinaryBallStructuringElement ${ITK_LIBRARIES})
