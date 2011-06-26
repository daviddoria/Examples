#include <vtkSmartPointer.h>
#include <vtkGraph.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkMutableDirectedGraph.h>

#include "vtkTestFilter.h"

void TestDirected();
void TestUndirected();

int main(int, char *[])
{
  TestDirected();
  TestUndirected();
  
  return EXIT_SUCCESS;
}

void TestDirected()
{
  
  vtkSmartPointer<vtkMutableDirectedGraph> g =
    vtkSmartPointer<vtkMutableDirectedGraph>::New();
  vtkIdType v1 = g->AddVertex();
  vtkIdType v2 = g->AddVertex();

  g->AddEdge(v1, v2);
  std::cout << "Input type: " << g->GetClassName() << std::endl;

  vtkSmartPointer<vtkTestFilter> filter =
    vtkSmartPointer<vtkTestFilter>::New();
  filter->SetInput(g);
  filter->Update();

  std::cout << "Output type: " << filter->GetOutput()->GetClassName() << std::endl;
  std::cout << "Output has " << filter->GetOutput()->GetNumberOfVertices() << " vertices." << std::endl;
  std::cout << std::endl;
}

void TestUndirected()
{
  std::cout << "TestUndirected" << std::endl;
  vtkSmartPointer<vtkMutableUndirectedGraph> g =
    vtkSmartPointer<vtkMutableUndirectedGraph>::New();
  vtkIdType v1 = g->AddVertex();
  vtkIdType v2 = g->AddVertex();

  g->AddEdge(v1, v2);
  std::cout << "Input type: " << g->GetClassName() << std::endl;

  vtkSmartPointer<vtkTestFilter> filter =
    vtkSmartPointer<vtkTestFilter>::New();
  filter->SetInput(g);
  filter->Update();

  std::cout << "Output type: " << filter->GetOutput()->GetClassName() << std::endl;
  std::cout << "Output has " << filter->GetOutput()->GetNumberOfVertices() << " vertices." << std::endl;
  std::cout << std::endl;
}