#include <vtkSmartPointer.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkGraphWriter.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> g = 
      vtkSmartPointer<vtkMutableUndirectedGraph>::New();

  vtkIdType v1 = g->AddVertex();
  vtkIdType v2 = g->AddVertex();

  g->AddEdge ( v1, v2 );

  std::cout << "Number of vertices: " << g->GetNumberOfVertices() << std::endl;
  std::cout << "Number of edges: " << g->GetNumberOfEdges() << std::endl;

  g->AddEdge ( v1, v2 );

  std::cout << "Number of vertices: " << g->GetNumberOfVertices() << std::endl;
  std::cout << "Number of edges: " << g->GetNumberOfEdges() << std::endl;
  
  vtkSmartPointer<vtkGraphWriter> writer = 
      vtkSmartPointer<vtkGraphWriter>::New();
  writer->SetInput(g);
  writer->SetFileName("graph.txt");
  writer->Write();
  
  return EXIT_SUCCESS;
}
