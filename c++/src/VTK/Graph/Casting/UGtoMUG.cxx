#include <vtkMutableUndirectedGraph.h>
#include <vtkUndirectedGraph.h>
#include <vtkSmartPointer.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> mug = 
    vtkSmartPointer<vtkMutableUndirectedGraph>::New();
  
  //create 3 vertices
  mug->AddVertex();
  mug->AddVertex();
  mug->AddVertex();
  
  std::cout << "MUG type: " << mug->GetClassName() << std::endl;
  std::cout << "MUG Vertices: " << mug->GetNumberOfVertices() << std::endl;
  
  vtkUndirectedGraph* ug = vtkUndirectedGraph::SafeDownCast(mug);
  
  std::cout << "UG type: " << ug->GetClassName() << std::endl;
  std::cout << "UG Vertices: " << ug->GetNumberOfVertices() << std::endl;
  
  vtkMutableUndirectedGraph* mug2 = vtkMutableUndirectedGraph::SafeDownCast(ug);
  std::cout << "MUG2 type: " << mug2->GetClassName() << std::endl;
  std::cout << "MUG2 Vertices: " << mug2->GetNumberOfVertices() << std::endl;
  
  return 0;
}
