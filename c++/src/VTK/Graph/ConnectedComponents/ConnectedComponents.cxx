#include <vtkSmartPointer.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkBoostConnectedComponents.h>
#include <vtkIntArray.h>
#include <vtkDataArray.h>
#include <vtkGraph.h>
#include <vtkDataSetAttributes.h>

int main ( int argc, char *argv[] )
{
  vtkSmartPointer<vtkMutableUndirectedGraph> g = 
      vtkSmartPointer<vtkMutableUndirectedGraph>::New();

  vtkIdType v1 = g->AddVertex();
  vtkIdType v2 = g->AddVertex();
  vtkIdType v3 = g->AddVertex();
  vtkIdType v4 = g->AddVertex();

  g->AddEdge(v1, v2);
  g->AddEdge(v3, v4);
  
  vtkSmartPointer<vtkBoostConnectedComponents> connectedComponents = 
      vtkSmartPointer<vtkBoostConnectedComponents>::New();
  connectedComponents->SetInput(g);
  connectedComponents->Update();
  
  vtkGraph* outputGraph = connectedComponents->GetOutput();

  vtkIntArray* components = vtkIntArray::SafeDownCast(outputGraph->GetVertexData()->GetArray("component"));
  
  for(unsigned int i = 0; i < components->GetNumberOfTuples(); i++)
    {
    int val = components->GetValue(i);
    std::cout << val << std::endl;
    }
  
  return EXIT_SUCCESS;
}
