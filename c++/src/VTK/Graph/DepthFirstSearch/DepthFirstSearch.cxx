#include <vtkMutableUndirectedGraph.h>
#include <vtkBoostPrimMinimumSpanningTree.h>
#include <vtkTree.h>
#include <vtkDoubleArray.h>
#include <vtkDataSetAttributes.h>
#include <vtkSmartPointer.h>
#include <vtkTreeDFSIterator.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> G = 
      vtkSmartPointer<vtkMutableUndirectedGraph>::New();
  
  //create 3 vertices
  vtkIdType V1 = G->AddVertex();
  vtkIdType V2 = G->AddVertex();
  vtkIdType V3 = G->AddVertex();
  
  //create a fully connected graph
  G->AddEdge(V1, V2);
  G->AddEdge(V2, V3);
  G->AddEdge(V1, V3);
  
  //create the edge weight array
  vtkSmartPointer<vtkDoubleArray> Weights = 
      vtkSmartPointer<vtkDoubleArray>::New();
  Weights->SetNumberOfComponents(1);
  Weights->SetName("Weights");
  
  //set the edge weights
  Weights->InsertNextValue(1.0);
  Weights->InsertNextValue(1.0);
  Weights->InsertNextValue(2.0);
  
  //add the edge weight array to the graph
  G->GetEdgeData()->AddArray(Weights);
  
  //output original graph info
  cout << "Original Graph" << vtkstd::endl << "----------" << endl;
  cout << "Number of vertices: " << G->GetNumberOfVertices() << endl;
  cout << "Number of edges: " << G->GetNumberOfEdges() << endl;
  
  //setup the minimum spanning tree filter
  vtkSmartPointer<vtkBoostPrimMinimumSpanningTree> MinimumSpanningTreeFilter = 
      vtkSmartPointer<vtkBoostPrimMinimumSpanningTree>::New();
  MinimumSpanningTreeFilter->SetOriginVertex(V1);
  MinimumSpanningTreeFilter->SetInput(G);
  MinimumSpanningTreeFilter->SetEdgeWeightArrayName("Weights");
  
  //compute the minimum spanning tree
  MinimumSpanningTreeFilter->Update();
  
  //get the output tree
  vtkSmartPointer<vtkTree> MinimumSpanningTree = 
      vtkSmartPointer<vtkTree>::New();
  MinimumSpanningTree->ShallowCopy(MinimumSpanningTreeFilter->GetOutput());
  
  //output information about the minimum spanning tree
  cout << endl;
  cout << "Minimum spanning tree" << endl << "----------" << endl;
  cout << "Number of vertices: " << MinimumSpanningTree->GetNumberOfVertices() << endl;
  cout << "Number of edges: " << MinimumSpanningTree->GetNumberOfEdges() << endl;
  
  vtkIdType root = MinimumSpanningTree->GetRoot();
  cout << "Root: " << root << endl;
  
  vtkSmartPointer<vtkTreeDFSIterator> DFS = 
      vtkSmartPointer<vtkTreeDFSIterator>::New();
  DFS->SetStartVertex(root);
  DFS->SetTree(MinimumSpanningTree);
  
  //traverse the tree in a depth first fashion
  while(DFS->HasNext())
    {
    vtkIdType NextVertex = DFS->Next();
    cout << "Next vertex: " << NextVertex << " level: " << MinimumSpanningTree->GetLevel(NextVertex) << endl;
    }

  return EXIT_SUCCESS;
}
