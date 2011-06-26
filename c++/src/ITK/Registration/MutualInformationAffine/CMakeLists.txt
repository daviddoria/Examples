cmake_minimum_required(VERSION 2.6)

PROJECT(MutualInformationAffine)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(MutualInformationAffine MutualInformationAffine.cxx)
TARGET_LINK_LIBRARIES(MutualInformationAffine ITKIO ITKNumerics)

