cmake_minimum_required(VERSION 2.6)

PROJECT(SetTargetProperties)

ADD_EXECUTABLE(SetTargetProperties SetTargetProperties.cxx)

#set_target_properties(SetTargetProperties PROPERTIES COMPILE_FLAGS "INTERACTIVE") # this does NOT work!

# The following two lines seem to do the same thing
#set_target_properties(SetTargetProperties PROPERTIES COMPILE_FLAGS "-DINTERACTIVE")
#set_target_properties(SetTargetProperties PROPERTIES COMPILE_DEFINITIONS "INTERACTIVE")