#include <vtkSmartPointer.h>
#include <vtkGraph.h>
#include <vtkMutableUndirectedGraph.h>

#include "vtkTestSource.h"

int main (int argc, char *argv[])
{
  vtkSmartPointer<vtkTestSource> source = vtkSmartPointer<vtkTestSource>::New();
  source->Update();
  
  vtkGraph* outputGraph = source->GetOutput();
  
  vtkstd::cout << "Output number of vertices: " << outputGraph->GetNumberOfVertices() << vtkstd::endl;
  
  return 0;
}
