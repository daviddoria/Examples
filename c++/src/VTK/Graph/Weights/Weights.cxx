#include <vtkMutableUndirectedGraph.h>
#include <vtkDoubleArray.h>
#include <vtkDataSetAttributes.h>
#include <vtkSmartPointer.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> g = 
      vtkSmartPointer<vtkMutableUndirectedGraph>::New();
  
  //create 3 vertices
  vtkIdType v1 = g->AddVertex();
  vtkIdType v2 = g->AddVertex();
  vtkIdType v3 = g->AddVertex();
  
  //create a fully connected graph
  g->AddEdge(v1, v2);
  g->AddEdge(v2, v3);
  g->AddEdge(v1, v3);
  
  //create the edge weight array
  vtkSmartPointer<vtkDoubleArray> weights = 
      vtkSmartPointer<vtkDoubleArray>::New();
  weights->SetNumberOfComponents(1);
  weights->SetName("Weights");
  
  //set the edge weights
  weights->InsertNextValue(1.0);
  weights->InsertNextValue(1.0);
  weights->InsertNextValue(2.0);
  
  //add the edge weight array to the graph
  g->GetEdgeData()->AddArray(weights);
  
  cout << "Number of Weights: " << vtkDoubleArray::SafeDownCast(g->GetEdgeData()->GetArray("Weights"))->GetNumberOfTuples() << endl;
  
  for(unsigned int i = 0; i < weights->GetNumberOfTuples(); i++)
    {
    double w = weights->GetValue(i);
    cout << "Weight " << i << " : " << w << endl;
    }
  
  //clear weights
  g->GetEdgeData()->RemoveArray("Weights");
    
  return EXIT_SUCCESS;
}
