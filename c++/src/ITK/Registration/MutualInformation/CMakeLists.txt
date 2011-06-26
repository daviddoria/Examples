cmake_minimum_required(VERSION 2.6)

PROJECT(MutualInformation)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(MutualInformation MutualInformation.cxx)
TARGET_LINK_LIBRARIES(MutualInformation ITKIO ITKNumerics)

