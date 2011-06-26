#include <vtkMutableUndirectedGraph.h>
#include <vtkGraph.h>
#include <vtkSmartPointer.h>
#include <vtkBoostConnectedComponents.h>

/*
//works
int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> MUG = vtkSmartPointer<vtkMutableUndirectedGraph>::New();
  
  //create 3 vertices
  MUG->AddVertex();
  MUG->AddVertex();
  MUG->AddVertex();
  
  vtkstd::cout << "MUG Vertices: " << MUG->GetNumberOfVertices() << vtkstd::endl;
  
  vtkSmartPointer<vtkBoostConnectedComponents> ConnectedComponents = vtkSmartPointer<vtkBoostConnectedComponents>::New();
  ConnectedComponents->SetInput(MUG);
  ConnectedComponents->Update();
  
  vtkGraph* ComponentsGraph = ConnectedComponents->GetOutput();
  vtkstd::cout << "ComponentsGraph Vertices: " << ComponentsGraph->GetNumberOfVertices() << vtkstd::endl;
  
  
  vtkMutableUndirectedGraph* DisconnectedGraph = vtkMutableUndirectedGraph::SafeDownCast(ComponentsGraph);
    
  vtkstd::cout << "DisconnectedGraph Vertices: " << DisconnectedGraph->GetNumberOfVertices() << vtkstd::endl;
  
  return 0;
}

*/

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> MUG = vtkSmartPointer<vtkMutableUndirectedGraph>::New();
  
  //create 3 vertices
  MUG->AddVertex();
  MUG->AddVertex();
  MUG->AddVertex();
  
  vtkGraph* G = vtkGraph::SafeDownCast(MUG);
  
  vtkstd::cout << "G Vertices: " << G->GetNumberOfVertices() << vtkstd::endl;
  
  vtkSmartPointer<vtkBoostConnectedComponents> ConnectedComponents = vtkSmartPointer<vtkBoostConnectedComponents>::New();
  ConnectedComponents->SetInput(G);
  ConnectedComponents->Update();
  
  vtkGraph* ComponentsGraph = ConnectedComponents->GetOutput();
  vtkstd::cout << "ComponentsGraph Vertices: " << ComponentsGraph->GetNumberOfVertices() << vtkstd::endl;
  vtkstd::cout << "ComponentsGraph class name: " << ComponentsGraph->GetClassName() << vtkstd::endl;
  
  vtkMutableUndirectedGraph* DisconnectedGraph = vtkMutableUndirectedGraph::SafeDownCast(ComponentsGraph);
    
  vtkstd::cout << "DisconnectedGraph Vertices: " << DisconnectedGraph->GetNumberOfVertices() << vtkstd::endl;
  
  return 0;
}
