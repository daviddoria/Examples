cmake_minimum_required(VERSION 2.6)

Project(ReadGraph)

FIND_PACKAGE(Boost)

LINK_DIRECTORIES(${LINK_DIRECTORIES} ${Boost_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIRS})

# ADD_EXECUTABLE(ReadGraph ReadGraph.cpp)
# target_link_libraries(ReadGraph boost_graph)

ADD_EXECUTABLE(FixInput FixInput.cpp)
target_link_libraries(FixInput boost_graph)
