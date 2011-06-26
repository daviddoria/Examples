cmake_minimum_required(VERSION 2.6)

INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} 
/home/doriad/src/Insight/Code/Review/
)

PROJECT(QuadEdgeMesh)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

##################
ADD_EXECUTABLE(QuadEdgeMesh QuadEdgeMesh.cpp)
TARGET_LINK_LIBRARIES(QuadEdgeMesh ITKIO 
ITKQuadEdgeMesh
)

##################
ADD_EXECUTABLE(QuadEdgeMeshWithTraits QuadEdgeMeshWithTraits.cpp)
TARGET_LINK_LIBRARIES(QuadEdgeMeshWithTraits ITKIO 
ITKQuadEdgeMesh
)

##################
ADD_EXECUTABLE(QuadEdgeMeshFrontIterator QuadEdgeMeshFrontIterator.cpp)
TARGET_LINK_LIBRARIES(QuadEdgeMeshFrontIterator ITKIO 
ITKQuadEdgeMesh
)

# ##################
# ADD_EXECUTABLE(QuadEdgeMeshFrontCompareIterator QuadEdgeMeshFrontCompareIterator.cpp)
# TARGET_LINK_LIBRARIES(QuadEdgeMeshFrontCompareIterator ITKIO 
# ITKQuadEdgeMesh)

##################
ADD_EXECUTABLE(MeshRead MeshRead.cpp)
TARGET_LINK_LIBRARIES(MeshRead ITKIO 
ITKQuadEdgeMesh
)

##################
ADD_EXECUTABLE(MeshDecimation MeshDecimation.cpp)
TARGET_LINK_LIBRARIES(MeshDecimation ITKIO 
ITKQuadEdgeMesh
)

##################
ADD_EXECUTABLE(GetEdgeInfo GetEdgeInfo.cpp)
TARGET_LINK_LIBRARIES(GetEdgeInfo ITKIO 
ITKQuadEdgeMesh
)

##################
ADD_EXECUTABLE(DuplicatePointsEdges DuplicatePointsEdges.cpp)
TARGET_LINK_LIBRARIES(DuplicatePointsEdges ITKIO 
ITKQuadEdgeMesh
)

##################
ADD_EXECUTABLE(TestEmptyMesh TestEmptyMesh.cpp)
TARGET_LINK_LIBRARIES(TestEmptyMesh ITKIO 
ITKQuadEdgeMesh
)

##################
ADD_EXECUTABLE(WriteMesh WriteMesh.cpp)
TARGET_LINK_LIBRARIES(WriteMesh ITKIO 
ITKQuadEdgeMesh
)