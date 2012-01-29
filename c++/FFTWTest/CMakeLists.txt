cmake_minimum_required(VERSION 2.6)

PROJECT(FFTWTest)

#SET(CMAKE_MODULE_PATH "/media/portable/Examples/c++/src/FFTWTest") #works
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})

find_package(FFTW REQUIRED)
#include(${FFTW_USE_FILE})

ADD_EXECUTABLE(FFTWTest FFTWTest.cpp )
TARGET_LINK_LIBRARIES(FFTWTest fftw3)