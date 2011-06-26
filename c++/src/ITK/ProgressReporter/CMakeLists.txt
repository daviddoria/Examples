cmake_minimum_required(VERSION 2.6)

PROJECT(ProgressReporter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ProgressReporter ProgressReporter.cxx)
TARGET_LINK_LIBRARIES(ProgressReporter ITKIO)