cmake_minimum_required(VERSION 2.6)

PROJECT(GTest)

#find_package(FindGTest REQUIRED)

include_directories(/home/doriad/src/gtest-1.6.0/include)
link_directories(/home/doriad/build/gtest)

ADD_EXECUTABLE(GTestWithMain GTestWithMain.cpp)
target_link_libraries(GTestWithMain gtest pthread)

ADD_EXECUTABLE(GTest GTest.cpp /home/doriad/src/gtest-1.6.0/src/gtest_main.cc)
target_link_libraries(GTest gtest pthread)