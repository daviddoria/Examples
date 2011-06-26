#include <vtkMutableUndirectedGraph.h>
#include <vtkUndirectedGraph.h>
#include <vtkSmartPointer.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> MUG = vtkSmartPointer<vtkMutableUndirectedGraph>::New();
  
  //create 3 vertices
  MUG->AddVertex();
  MUG->AddVertex();
  MUG->AddVertex();
  
  vtkstd::cout << "MUG Vertices: " << MUG->GetNumberOfVertices() << vtkstd::endl;
  
  vtkUndirectedGraph* UG = vtkUndirectedGraph::SafeDownCast(MUG);
  
  vtkstd::cout << "UG Vertices: " << UG->GetNumberOfVertices() << vtkstd::endl;
  
  return 0;
}
