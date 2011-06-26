cmake_minimum_required(VERSION 2.6)

PROJECT(SetTargetProperties)

ADD_EXECUTABLE(SetTargetProperties SetTargetProperties.cxx)
set_target_properties(SetTargetProperties PROPERTIES COMPILE_FLAGS "-DUNIX")