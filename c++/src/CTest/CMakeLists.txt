cmake_minimum_required(VERSION 2.6)

PROJECT(CTestDemo)

ADD_EXECUTABLE(Test test.cpp)

ENABLE_TESTING()

ADD_TEST(SampleTest Test)

#run with simply (from the tests directory):
#ctest

#to see output (std::cout) as well as summary:
#ctest -V
