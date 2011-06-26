#include <vtkMutableUndirectedGraph.h>
#include <vtkSmartPointer.h>
#include <vtkAdjacentVertexIterator.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> g = 
    vtkSmartPointer<vtkMutableUndirectedGraph>::New();
  
  //create 3 vertices
  vtkIdType v1 = g->AddVertex();
  vtkIdType v2 = g->AddVertex();
  vtkIdType v3 = g->AddVertex();
  
  //create a fully connected graph
  g->AddEdge(v1, v2);
  g->AddEdge(v2, v3);
  g->AddEdge(v1, v3);
  
  vtkSmartPointer<vtkAdjacentVertexIterator> iterator = 
    vtkSmartPointer<vtkAdjacentVertexIterator>::New();
  g->GetAdjacentVertices(0, iterator);
  
  while(iterator->HasNext())
  {
    vtkIdType nextVertex = iterator->Next();
    std::cout << "Next vertex: " << nextVertex << std::endl;
  }
  
  return 0;
}
