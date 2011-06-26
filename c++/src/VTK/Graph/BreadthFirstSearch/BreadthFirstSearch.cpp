#include <vtkSmartPointer.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkIntArray.h>
#include <vtkBoostBreadthFirstSearch.h>
#include <vtkTree.h>
#include <vtkDoubleArray.h>
#include <vtkDataSetAttributes.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> g = 
      vtkSmartPointer<vtkMutableUndirectedGraph>::New();
  
  //create 3 vertices
  vtkIdType v1 = g->AddVertex();
  vtkIdType v2 = g->AddVertex();
  vtkIdType v3 = g->AddVertex();
  vtkIdType v4 = g->AddVertex();
  
  //create a graph
  g->AddEdge(v1, v2);
  g->AddEdge(v1, v3);
  g->AddEdge(v2, v4);
  
  vtkSmartPointer<vtkBoostBreadthFirstSearch> BFS = 
      vtkSmartPointer<vtkBoostBreadthFirstSearch>::New();
  BFS->SetOriginVertex(v1);
  BFS->SetInput(g);
  BFS->Update();
  
  vtkIntArray* level = vtkIntArray::SafeDownCast(BFS->GetOutput()->GetVertexData()->GetArray("BFS"));
  for(vtkIdType i = 0; i < level->GetNumberOfTuples(); i++)
    {
    cout << "id " << i << " : " << level->GetValue(i) << endl;
    }
  return EXIT_SUCCESS;
}
