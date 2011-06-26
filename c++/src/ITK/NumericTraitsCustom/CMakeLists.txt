cmake_minimum_required(VERSION 2.6)

PROJECT(NumericTraitsCustom)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(NumericTraitsCustom NumericTraitsCustom.cxx)
TARGET_LINK_LIBRARIES(NumericTraitsCustom ${ITK_LIBRARIES})
