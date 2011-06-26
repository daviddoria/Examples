cmake_minimum_required(VERSION 2.6)

Project(VSLBasics)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

ADD_EXECUTABLE(VSLBasics VSLBasics.cpp)
TARGET_LINK_LIBRARIES(VSLBasics
vil vnl vnl_io vsl )

ADD_EXECUTABLE(CustomClass CustomClass.cpp TestClass.cpp)
TARGET_LINK_LIBRARIES(CustomClass
vil vnl vnl_io vsl )
