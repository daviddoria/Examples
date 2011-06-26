#include <vtkSmartPointer.h>
#include <vtkDataSetAttributes.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkPBGLShortestPaths.h>
#include <vtkDoubleArray.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> g = 
    vtkSmartPointer<vtkMutableUndirectedGraph>::New();
  
  // Create vertices
  vtkIdType v0 = g->AddVertex();
  vtkIdType v1 = g->AddVertex();
  vtkIdType v2 = g->AddVertex();
  vtkIdType v3 = g->AddVertex();
  
  g->AddEdge(v0,v1);
  g->AddEdge(v1,v2);
  g->AddEdge(v2,v3);
  
  // Create the edge weight array
  vtkSmartPointer<vtkDoubleArray> weights = 
    vtkSmartPointer<vtkDoubleArray>::New();
  weights->SetNumberOfComponents(1);
  weights->SetName("Weights");
 
  // Set the edge weights
  weights->InsertNextValue(1.0);
  weights->InsertNextValue(1.0);
  weights->InsertNextValue(2.0);
  
  g->GetEdgeData()->AddArray(weights);
   
  vtkSmartPointer<vtkPBGLShortestPaths> shortestPathsFilter = 
    vtkSmartPointer<vtkPBGLShortestPaths>::New();
  shortestPathsFilter->SetInputConnection(g->GetProducerPort());
  shortestPathsFilter->SetOriginVertex(v0);
  shortestPathsFilter->SetEdgeWeightArrayName("Weights");
  shortestPathsFilter->Update();
  
  vtkDoubleArray* pathLengths = vtkDoubleArray::SafeDownCast
	(shortestPathsFilter->GetOutput()->GetVertexData()->GetAbstractArray("PathLength"));
 
  for(vtkIdType i = 0; i < pathLengths->GetNumberOfTuples(); i++)
    {
    std::cout << "Distance from v0: " << pathLengths->GetValue(i) << std::endl;
    }
    
  return EXIT_SUCCESS;
}
