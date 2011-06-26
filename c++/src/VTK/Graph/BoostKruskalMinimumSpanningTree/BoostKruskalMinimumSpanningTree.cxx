#include <vtkSmartPointer.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkBoostKruskalMinimumSpanningTree.h>
#include <vtkTree.h>
#include <vtkDoubleArray.h>
#include <vtkDataSetAttributes.h>
#include <vtkExtractSelectedGraph.h>

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
  
  //output original graph info
  cout << "Number of vertices: " << g->GetNumberOfVertices() << endl;
  cout << "Number of edges: " << g->GetNumberOfEdges() << endl;
  
  //setup the minimum spanning tree filter
  vtkSmartPointer<vtkBoostKruskalMinimumSpanningTree> minimumSpanningTreeFilter = 
      vtkSmartPointer<vtkBoostKruskalMinimumSpanningTree>::New();
  minimumSpanningTreeFilter->SetInput(g);
  minimumSpanningTreeFilter->SetEdgeWeightArrayName("Weights");
  minimumSpanningTreeFilter->Update();
  
  
  vtkSmartPointer<vtkExtractSelectedGraph> extractSelectedGraph = 
      vtkSmartPointer<vtkExtractSelectedGraph>::New();
  extractSelectedGraph->SetInput(0, g);
  extractSelectedGraph->SetInput(1, minimumSpanningTreeFilter->GetOutput());
  extractSelectedGraph->Update();
  
  //get the output tree
  vtkSmartPointer<vtkTree> minimumSpanningTree = 
      vtkSmartPointer<vtkTree>::New();
  minimumSpanningTree->ShallowCopy(extractSelectedGraph->GetOutput());
  
  //output information about the minimum spanning tree
  cout << "Number of vertices: " << minimumSpanningTree->GetNumberOfVertices() << endl;
  cout << "Number of edges: " << minimumSpanningTree->GetNumberOfEdges() << endl;
  
  return EXIT_SUCCESS;
}
