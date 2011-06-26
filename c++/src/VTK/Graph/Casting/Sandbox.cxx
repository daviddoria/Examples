#include <vtkSmartPointer.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkMutableDirectedGraph.h>
#include <vtkGraph.h>
#include <vtkDirectedGraph.h>
#include <vtkUndirectedGraph.h>
#include <vtkFloatArray.h>
#include <vtkDataSetAttributes.h>

int main(int argc, char *argv[])
{
  /*
  vtkSmartPointer<vtkMutableUndirectedGraph> mug = 
    vtkSmartPointer<vtkMutableUndirectedGraph>::New();
  std::cout << mug->GetClassName() << std::endl;
  std::cout << mug << std::endl;
    
  vtkSmartPointer<vtkUndirectedGraph> ug = vtkUndirectedGraph::SafeDownCast(mug);
  
  std::cout << ug->GetClassName() << std::endl;
  std::cout << ug << std::endl;
  */
  
  /*
  vtkSmartPointer<vtkUndirectedGraph> ug = 
    vtkSmartPointer<vtkUndirectedGraph>::New();
  std::cout << ug->GetClassName() << std::endl;
  std::cout << ug << std::endl;
    
  vtkSmartPointer<vtkMutableUndirectedGraph> mug = vtkMutableUndirectedGraph::SafeDownCast(ug);
  
  std::cout << mug->GetClassName() << std::endl;
  std::cout << mug << std::endl;
  */
  
  vtkSmartPointer<vtkUndirectedGraph> ug = 
    vtkSmartPointer<vtkUndirectedGraph>::New();
  
  vtkSmartPointer<vtkFloatArray> weights = 
    vtkSmartPointer<vtkFloatArray>::New();
  weights->SetNumberOfComponents(1);
  weights->SetNumberOfTuples(ug->GetNumberOfEdges());
  weights->SetName("Weights");
  
  ug->GetEdgeData()->AddArray(weights);
  
  
  return 0;
}
