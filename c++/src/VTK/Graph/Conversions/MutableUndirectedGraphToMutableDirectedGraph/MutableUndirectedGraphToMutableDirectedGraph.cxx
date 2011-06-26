#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkMutableDirectedGraph.h>
#include <vtkMutableUndirectedGraph.h>

int main (int, char *[])
{
  
  //create a graph
  vtkSmartPointer<vtkMutableUndirectedGraph> mug =
    vtkSmartPointer<vtkMutableUndirectedGraph>::New();

  //add 4 vertices to the graph
  vtkIdType v1 = mug->AddVertex();
  vtkIdType v2 = mug->AddVertex();
  vtkIdType v3 = mug->AddVertex();
  vtkIdType v4 = mug->AddVertex();

  //add 3 edges to the graph
  mug->AddEdge ( v1, v2 );
  mug->AddEdge ( v1, v3 );
  mug->AddEdge ( v1, v4 );

  //create 4 points - one for each vertex
  vtkSmartPointer<vtkPoints> points =
      vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(0.0, 0.0, 0.0);
  points->InsertNextPoint(1.0, 0.0, 0.0);
  points->InsertNextPoint(0.0, 1.0, 0.0);
  points->InsertNextPoint(0.0, 0.0, 2.0);

  //add the coordinates of the points to the graph
  mug->SetPoints(points);

  std::cout << "MUG: " << std::endl;
  std::cout << "Type: " << mug->GetClassName() << std::endl;
  std::cout << "Vertices: " << mug->GetNumberOfVertices() << std::endl;
  std::cout << "Edges: " << mug->GetNumberOfEdges() << std::endl;

  vtkSmartPointer<vtkMutableDirectedGraph> mdg =
      vtkSmartPointer<vtkMutableDirectedGraph>::New();
  if(!mdg->CheckedShallowCopy(mug))
    {
    std::cerr << "Could not convert!" << std::endl;
    return EXIT_FAILURE;
    }

  std::cout << "MDG: " << std::endl;
  std::cout << "Type: " << mdg->GetClassName() << std::endl;
  std::cout << "Vertices: " << mdg->GetNumberOfVertices() << std::endl;
  std::cout << "Edges: " << mdg->GetNumberOfEdges() << std::endl;


  return EXIT_SUCCESS;
}
