cmake_minimum_required(VERSION 2.6)
 
PROJECT(ParticleReader)
 
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
 
ADD_EXECUTABLE(ParticleReader ParticleReader.cxx)
 
TARGET_LINK_LIBRARIES(ParticleReader vtkHybrid)
