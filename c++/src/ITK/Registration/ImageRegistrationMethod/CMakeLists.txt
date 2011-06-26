cmake_minimum_required(VERSION 2.6)

PROJECT(Registration)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(Registration Registration.cpp)
TARGET_LINK_LIBRARIES(Registration ITKIO ITKNumerics)

