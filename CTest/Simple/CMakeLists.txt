cmake_minimum_required(VERSION 2.6)

project(simple)
ENABLE_TESTING()

add_executable(simple simple.cpp)
add_test(NAME SimpleTest COMMAND simple)