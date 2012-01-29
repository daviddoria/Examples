cmake_minimum_required(VERSION 2.6)

Project(BoostRegex)

ADD_EXECUTABLE(BoostRegex Regex.cpp)
TARGET_LINK_LIBRARIES(BoostRegex boost_regex)

ADD_EXECUTABLE(working working.cpp)
TARGET_LINK_LIBRARIES(working boost_regex)
