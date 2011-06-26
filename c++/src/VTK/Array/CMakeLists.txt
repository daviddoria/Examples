cmake_minimum_required(VERSION 2.6)

PROJECT(Array)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Array Array.cxx)
TARGET_LINK_LIBRARIES(Array vtkHybrid)


#ADD_EXECUTABLE(FixedLength FixedLength.cxx)
#TARGET_LINK_LIBRARIES(FixedLength vtkHybrid)

#ADD_EXECUTABLE(VectorArrayKnownLength VectorArrayKnownLength.cxx)
#TARGET_LINK_LIBRARIES(VectorArrayKnownLength vtkHybrid)

#ADD_EXECUTABLE(VectorArrayUnknownLength VectorArrayUnknownLength.cxx)
#TARGET_LINK_LIBRARIES(VectorArrayUnknownLength vtkHybrid)
