cmake_minimum_required(VERSION 2.6)

PROJECT(TransformData)

#########################################3
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

######################################
ADD_EXECUTABLE(TransformData TransformData.cxx)
TARGET_LINK_LIBRARIES(TransformData vtkHybrid)

######################################
ADD_EXECUTABLE(TransformOrder TransformOrder.cxx)
TARGET_LINK_LIBRARIES(TransformOrder vtkHybrid)

######################################
ADD_EXECUTABLE(TransformOrderDemo TransformOrderDemo.cxx)
TARGET_LINK_LIBRARIES(TransformOrderDemo vtkHybrid)

######################################
ADD_EXECUTABLE(Transform Transform.cxx)
TARGET_LINK_LIBRARIES(Transform vtkHybrid)
