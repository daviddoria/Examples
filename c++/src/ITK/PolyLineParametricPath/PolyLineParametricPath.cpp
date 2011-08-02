#include "itkPolyLineParametricPath.h"

int main(int argc, char *argv[])
{
  typedef itk::PolyLineParametricPath< 2 > PathType;
  
  PathType::Pointer path = PathType::New();
  path->Initialize();

  typedef PathType::ContinuousIndexType    ContinuousIndexType;

  // Create a line
  for(unsigned int i = 0; i < 20; ++i)
    {
    ContinuousIndexType cindex;
    cindex[0] = 0;
    cindex[1] = i;
    path->AddVertex( cindex );
    }
        
  const PathType::VertexListType * vertexList = path->GetVertexList ();
  
  for(unsigned int i = 0; i < vertexList->Size(); ++i)
    {
    std::cout << vertexList->GetElement(i) << std::endl;
    }
    
  return EXIT_SUCCESS;
}
