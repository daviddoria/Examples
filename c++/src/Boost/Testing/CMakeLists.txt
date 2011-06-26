cmake_minimum_required(VERSION 2.6)

Project(Testing)

#######################################
set(Boost_USE_MULTITHREADED ON)
FIND_PACKAGE(Boost 1.38 COMPONENTS unit_test_framework required)

INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${LINK_DIRECTORIES} ${Boost_LIBRARY_DIRS})

ADD_EXECUTABLE(Testing Testing.cpp)

TARGET_LINK_LIBRARIES(Testing 
#boost_unit_test_framework-mt #cant use this - undefined reference to main
boost_unit_test_framework-mt-s
)
