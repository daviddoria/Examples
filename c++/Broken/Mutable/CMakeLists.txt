cmake_minimum_required(VERSION 2.6)

Project(Test)

###############################
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

###############################
FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

#######################################
set(Boost_USE_MULTITHREADED ON) # which is the default
FIND_PACKAGE(Boost 1.38 COMPONENTS program_options required)


#INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIRS})
#LINK_DIRECTORIES(${LINK_DIRECTORIES} ${Boost_LIBRARY_DIRS})

ADD_EXECUTABLE(Test Test.cpp)
#target_link_libraries(Test boost_program_options-mt vtkHybrid vgl)
