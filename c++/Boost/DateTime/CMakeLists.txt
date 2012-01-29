cmake_minimum_required(VERSION 2.6)

Project(DateTime)

FIND_PACKAGE(Boost 1.38 COMPONENTS date_time required)
 #-lboost_date_time-gcc-1_31 
INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${LINK_DIRECTORIES} ${Boost_LIBRARY_DIRS})

ADD_EXECUTABLE(DateTime DateTime.cpp)
target_link_libraries(DateTime boost_date_time)

