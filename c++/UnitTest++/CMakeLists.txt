cmake_minimum_required(VERSION 2.6)

PROJECT(Test)
include_directories(/home/doriad/src/UnitTest++/src)
link_directories(/home/doriad/build/UnitTest++/)
ADD_EXECUTABLE(Test UnitTest++.cpp )
target_link_libraries(Test UnitTest++)