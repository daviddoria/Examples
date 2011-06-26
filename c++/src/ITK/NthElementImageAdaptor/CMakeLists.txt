cmake_minimum_required(VERSION 2.6)

PROJECT(NthElementImageAdaptor)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(NthElementImageAdaptor NthElementImageAdaptor.cxx)
TARGET_LINK_LIBRARIES(NthElementImageAdaptor ${ITK_LIBRARIES})

ADD_EXECUTABLE(ProcessingNthElement ProcessingNthElement.cxx)
TARGET_LINK_LIBRARIES(ProcessingNthElement ${ITK_LIBRARIES})

# ADD_EXECUTABLE(NthElementOfVectorImage NthElementOfVectorImage.cxx)
# TARGET_LINK_LIBRARIES(NthElementOfVectorImage ${ITK_LIBRARIES})
