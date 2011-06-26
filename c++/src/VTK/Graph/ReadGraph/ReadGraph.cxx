#include <vtkSmartPointer.h>
//#include <vtkMutableUndirectedGraph.h>
#include <vtkGraph.h>
#include <vtkGraphReader.h>

int main ( int argc, char *argv[] )
{
  vtkSmartPointer<vtkGraphReader> reader = 
      vtkSmartPointer<vtkGraphReader>::New();
  reader->SetFileName("graph.txt");
  reader->Update();
  
  vtkGraph* g = reader->GetOutput();

  cout << "Number of vertices: " << g->GetNumberOfVertices() << endl;
  cout << "Number of edges: " << g->GetNumberOfEdges() << endl;
  
  return EXIT_SUCCESS;
}
