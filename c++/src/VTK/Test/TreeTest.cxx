#include <vtkSmartPointer.h>
#include <vtkDoubleArray.h>
#include <vtkDataSetAttributes.h>
#include <vtkTree.h>
#include <vtkMutableDirectedGraph.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkMutableDirectedGraph> dg = 
      vtkSmartPointer<vtkMutableDirectedGraph>::New();
  
  vtkIdType v1 = dg->AddVertex();
  vtkIdType v2 = dg->AddChild(v1);
  
  vtkSmartPointer<vtkDoubleArray> weights = 
      vtkSmartPointer<vtkDoubleArray>::New();
  weights->SetNumberOfComponents(1);
  weights->SetName("Weights");
 
  //set the edge weights
  weights->InsertNextValue(1.0);
  weights->InsertNextValue(2.0);
  
  //add the edge weight array to the graph
  dg->GetEdgeData()->AddArray(weights);
 
  cout << "Number of Weights: " << vtkDoubleArray::SafeDownCast(dg->GetEdgeData()->GetArray("Weights"))->GetNumberOfTuples() << endl;
 
  for(unsigned int i = 0; i < weights->GetNumberOfTuples(); i++)
    {
    double w = weights->GetValue(i);
    cout << "Weight " << i << " : " << w << endl;
    }
  cout << "dg has " << dg->GetNumberOfVertices() << " vertices." << endl;
  
  vtkSmartPointer<vtkTree> tree = 
      vtkSmartPointer<vtkTree>::New();
  bool success = tree->CheckedShallowCopy(dg);
  cout << "Success? " << success << endl;
  
  cout << "tree has " << tree->GetNumberOfVertices() << " vertices." << endl;
  
  vtkSmartPointer<vtkMutableDirectedGraph> dg2 = 
      vtkSmartPointer<vtkMutableDirectedGraph>::New();
  
  dg2->DeepCopy(tree);
  
  cout << "dg2 has " << dg2->GetNumberOfVertices() << " vertices." << endl;
  cout << "dg2 has: " << vtkDoubleArray::SafeDownCast(dg2->GetEdgeData()->GetArray("Weights"))->GetNumberOfTuples() << " weights." << endl;
  return EXIT_SUCCESS;
}