cmake_minimum_required(VERSION 2.6)

PROJECT(AddTargetDefinitions)

ADD_EXECUTABLE(AddTargetDefinitions AddTargetDefinitions.cxx)

# This is apparently no longer a valid command?
#add_target_definitions(AddTargetDefinitions INTERACTIVE)