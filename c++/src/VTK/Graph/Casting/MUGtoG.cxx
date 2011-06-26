#include <vtkMutableUndirectedGraph.h>
#include <vtkGraph.h>
#include <vtkSmartPointer.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> MUG = vtkSmartPointer<vtkMutableUndirectedGraph>::New();
  
  //create 3 vertices
  MUG->AddVertex();
  MUG->AddVertex();
  MUG->AddVertex();
  
  vtkstd::cout << "MUG Vertices: " << MUG->GetNumberOfVertices() << vtkstd::endl;
  
  vtkGraph* G = vtkGraph::SafeDownCast(MUG);
  
  vtkstd::cout << "G Vertices: " << G->GetNumberOfVertices() << vtkstd::endl;
  
  return 0;
}
