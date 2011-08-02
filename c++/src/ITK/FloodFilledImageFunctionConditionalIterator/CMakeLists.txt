cmake_minimum_required(VERSION 2.6)

PROJECT(FloodFilledImageFunctionConditionalIterator)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(FloodFilledImageFunctionConditionalIterator FloodFilledImageFunctionConditionalIterator.cxx)
TARGET_LINK_LIBRARIES(FloodFilledImageFunctionConditionalIterator ${ITK_LIBRARIES})

