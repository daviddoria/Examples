cmake_minimum_required(VERSION 2.6)

PROJECT(Casting)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})


ADD_EXECUTABLE(ConnectedComponents ConnectedComponents.cxx)
TARGET_LINK_LIBRARIES(ConnectedComponents vtkHybrid vtkInfovis)

#ADD_EXECUTABLE(Casting Casting.cxx)
#TARGET_LINK_LIBRARIES(Casting vtkHybrid vtkInfovis)

# ADD_EXECUTABLE(GtoMUG GtoMUG.cxx)
# TARGET_LINK_LIBRARIES(GtoMUG vtkHybrid vtkInfovis)
# 
# ADD_EXECUTABLE(MUGtoG MUGtoG.cxx)
# TARGET_LINK_LIBRARIES(MUGtoG vtkHybrid vtkInfovis)
# 
ADD_EXECUTABLE(MUGtoUG MUGtoUG.cxx)
TARGET_LINK_LIBRARIES(MUGtoUG vtkHybrid vtkInfovis)

ADD_EXECUTABLE(UGtoMUG UGtoMUG.cxx)
TARGET_LINK_LIBRARIES(UGtoMUG vtkHybrid vtkInfovis)

ADD_EXECUTABLE(Sandbox Sandbox.cxx)
TARGET_LINK_LIBRARIES(Sandbox vtkHybrid vtkInfovis)
