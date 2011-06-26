cmake_minimum_required(VERSION 2.6)

PROJECT(WhiteTopHatImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(WhiteTopHatImageFilter WhiteTopHatImageFilter.cxx)
TARGET_LINK_LIBRARIES(WhiteTopHatImageFilter ${ITK_LIBRARIES})

