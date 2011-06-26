cmake_minimum_required(VERSION 2.6)

Project(ReadGraph)

FIND_PACKAGE(Boost)

LINK_DIRECTORIES(${LINK_DIRECTORIES} ${Boost_LIBRARY_DIRS})

ADD_EXECUTABLE(ReadGraph ReadGraph.cpp)
target_link_libraries(ReadGraph boost_graph)

