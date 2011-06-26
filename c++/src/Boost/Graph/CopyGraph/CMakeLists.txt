cmake_minimum_required(VERSION 2.6)

Project(CopyGraph)

FIND_PACKAGE(Boost)

LINK_DIRECTORIES(${LINK_DIRECTORIES} ${Boost_LIBRARY_DIRS})

ADD_EXECUTABLE(CopyGraph CopyGraph.cpp)
target_link_libraries(CopyGraph boost_graph)

