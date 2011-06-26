#include <vtkSmartPointer.h>
#include <vtkMutableUndirectedGraph.h>

int main ( int argc, char *argv[] )
{
  vtkSmartPointer<vtkMutableUndirectedGraph> g = 
      vtkSmartPointer<vtkMutableUndirectedGraph>::New();

  vtkIdType v1 = g->AddVertex();
  vtkIdType v2 = g->AddVertex();

  g->AddEdge ( v1, v2 );

  cout << "Number of vertices: " << g->GetNumberOfVertices() << endl;
  cout << "Number of edges: " << g->GetNumberOfEdges() << endl;

  //you can use GetAdjacentVertices(vtkIdType v, vtkAdjacentVertexIterator *it) 
  //to test if two vertices are connected before adding an edge to prevent duplicate edges

  g->AddEdge ( v1, v2 );

  cout << "Number of vertices: " << g->GetNumberOfVertices() << endl;
  cout << "Number of edges: " << g->GetNumberOfEdges() << endl;
  
  return EXIT_SUCCESS;
}
