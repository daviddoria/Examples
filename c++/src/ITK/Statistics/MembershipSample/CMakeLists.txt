cmake_minimum_required(VERSION 2.6)

PROJECT(MembershipSample)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(MembershipSample MembershipSample.cxx)
TARGET_LINK_LIBRARIES(MembershipSample
ITKBasicFilters ITKCommon ITKIO)
