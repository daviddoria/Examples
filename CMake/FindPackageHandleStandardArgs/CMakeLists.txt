cmake_minimum_required(VERSION 2.6)

PROJECT(Test)

ADD_EXECUTABLE(Test Test.cxx)

find_package(FFTW REQUIRED)

if(NOT ${FFTW_FOUND})
  MESSAGE(FATAL_ERROR "FFTW3 was not found!")
else()
  MESSAGE("FFTW3 was found.")
endif()
