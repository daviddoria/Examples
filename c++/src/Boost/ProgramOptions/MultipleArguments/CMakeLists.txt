cmake_minimum_required(VERSION 2.6)

Project(MultipleArguments)

set(Boost_USE_MULTITHREADED ON) # which is the default
FIND_PACKAGE(Boost 1.38 COMPONENTS program_options required)

INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${LINK_DIRECTORIES} ${Boost_LIBRARY_DIRS})

ADD_EXECUTABLE(MultipleArguments MultipleArguments.cpp)
target_link_libraries(MultipleArguments boost_program_options-mt)
