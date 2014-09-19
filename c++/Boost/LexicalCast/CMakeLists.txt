cmake_minimum_required(VERSION 2.6)

project(LexicalCast)

find_package(Boost)
include_directories(${Boost_INCLUDE_DIRS})

add_executable( StringToValue StringToValue.cxx )
add_executable( ValueToString ValueToString.cxx )
