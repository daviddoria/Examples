#include <vtkMutableUndirectedGraph.h>
#include <vtkBoostPrimMinimumSpanningTree.h>
#include <vtkTree.h>
#include <vtkDoubleArray.h>
#include <vtkDataSetAttributes.h>
#include <vtkSmartPointer.h>
#include <vtkAdjacentVertexIterator.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> G = vtkSmartPointer<vtkMutableUndirectedGraph>::New();
  
  //create 3 vertices
  vtkIdType V1 = G->AddVertex();
  vtkIdType V2 = G->AddVertex();
  vtkIdType V3 = G->AddVertex();
  
  //create a fully connected graph
  G->AddEdge(V1, V2);
  G->AddEdge(V2, V3);
  G->AddEdge(V1, V3);
  
  //create the edge weight array
  vtkSmartPointer<vtkDoubleArray> Weights = vtkSmartPointer<vtkDoubleArray>::New();
  Weights->SetNumberOfComponents(1);
  Weights->SetName("Weights");
  
  //set the edge weights
  Weights->InsertNextValue(1.0);
  Weights->InsertNextValue(1.0);
  Weights->InsertNextValue(2.0);
  
  //add the edge weight array to the graph
  G->GetEdgeData()->AddArray(Weights);
  
  //output original graph info
  vtkstd::cout << "Original Graph" << vtkstd::endl << "----------" << vtkstd::endl;
  vtkstd::cout << "Number of vertices: " << G->GetNumberOfVertices() << vtkstd::endl;
  vtkstd::cout << "Number of edges: " << G->GetNumberOfEdges() << vtkstd::endl;
  
  //setup the minimum spanning tree filter
  vtkSmartPointer<vtkBoostPrimMinimumSpanningTree> MinimumSpanningTreeFilter = vtkSmartPointer<vtkBoostPrimMinimumSpanningTree>::New();
  MinimumSpanningTreeFilter->SetOriginVertex(V1);
  MinimumSpanningTreeFilter->SetInput(G);
  MinimumSpanningTreeFilter->SetEdgeWeightArrayName("Weights");
  
  //compute the minimum spanning tree
  MinimumSpanningTreeFilter->Update();
  
  //get the output tree
  vtkSmartPointer<vtkTree> MinimumSpanningTree = vtkSmartPointer<vtkTree>::New();
  MinimumSpanningTree->ShallowCopy(MinimumSpanningTreeFilter->GetOutput());
  
  //output information about the minimum spanning tree
  vtkstd::cout << vtkstd::endl;
  vtkstd::cout << "Minimum spanning tree" << vtkstd::endl << "----------" << vtkstd::endl;
  vtkstd::cout << "Number of vertices: " << MinimumSpanningTree->GetNumberOfVertices() << vtkstd::endl;
  vtkstd::cout << "Number of edges: " << MinimumSpanningTree->GetNumberOfEdges() << vtkstd::endl;
  
  vtkIdType root = MinimumSpanningTree->GetRoot();
  vtkstd::cout << "Root: " << root << vtkstd::endl;
  
  vtkSmartPointer<vtkAdjacentVertexIterator> iterator = vtkSmartPointer<vtkAdjacentVertexIterator>::New();
  MinimumSpanningTree->GetChildren(root, iterator);

  vtkstd::cout << vtkstd::endl;
  vtkstd::cout << "Tree traversal" << vtkstd::endl << "--------" << vtkstd::endl;
  vtkstd::cout << "Root: " << iterator->GetVertex() << vtkstd::endl;
  vtkstd::cout << "Root children: " << MinimumSpanningTree->GetNumberOfChildren(root) << vtkstd::endl;
  
  //output all children - if the node has a next vertex, get it and output its ID
  while(iterator->HasNext())
  {
    vtkIdType NextVertex = iterator->Next();
    if(MinimumSpanningTree->GetNumberOfChildren(NextVertex) > 0)
    {
      
    }
    vtkstd::cout << "Next vertex: " << NextVertex << vtkstd::endl;
    vtkstd::cout << "Next vertex level: " << MinimumSpanningTree->GetLevel(NextVertex) << vtkstd::endl;
  }
  
  return 0;
}
