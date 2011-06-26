#include <vtkSmartPointer.h>
#include <vtkGraph.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkMutableDirectedGraph.h>

#include "vtkTestFilter.h"

int main (int argc, char *argv[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> inputGraph =
    vtkSmartPointer<vtkMutableUndirectedGraph>::New();

  vtkSmartPointer<vtkTestFilter> filter = 
    vtkSmartPointer<vtkTestFilter>::New();
  filter->SetOutputType("ug");  
  filter->SetInputConnection(inputGraph->GetProducerPort());
  filter->Update();

  return EXIT_SUCCESS;
}
