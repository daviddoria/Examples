#include <vtkMutableDirectedGraph.h>
#include <vtkBoostPrimMinimumSpanningTree.h>
#include <vtkBoostBreadthFirstSearch.h>
#include <vtkTree.h>
#include <vtkDoubleArray.h>
#include <vtkDataSetAttributes.h>
#include <vtkSmartPointer.h>

#include <vtkTreeBFSIterator.h>
#include <vtkTreeDFSIterator.h>
 
void DFS(vtkSmartPointer<vtkTree> tree);
void BFS(vtkSmartPointer<vtkTree> tree);

int main(int, char *[])
{
  vtkSmartPointer<vtkMutableDirectedGraph> g = 
    vtkSmartPointer<vtkMutableDirectedGraph>::New();
 
  //create 3 vertices
  vtkIdType v0 = g->AddVertex();
  vtkIdType v1 = g->AddVertex();
  vtkIdType v2 = g->AddVertex();
  vtkIdType v3 = g->AddVertex();
 
  //create a fully connected graph
  g->AddEdge(v0, v1);
  g->AddEdge(v0, v2);
  g->AddEdge(v1, v3);
  
  vtkSmartPointer<vtkTree> tree = 
      vtkSmartPointer<vtkTree>::New();
  tree->CheckedShallowCopy(g);
  
  DFS(tree);
  
  BFS(tree);
  
  return EXIT_SUCCESS;
}

void DFS(vtkSmartPointer<vtkTree> tree)
{
  //this should produce 0, 1, 3, 2
  
  cout << "DFS" << endl << "----------" << endl;
  
  vtkIdType root = tree->GetRoot();
  
  vtkSmartPointer<vtkTreeDFSIterator> dfsIterator = 
      vtkSmartPointer<vtkTreeDFSIterator>::New();
  dfsIterator->SetStartVertex(root);
  dfsIterator->SetTree(tree);
 
  //traverse the tree in a depth first fashion
  while(dfsIterator->HasNext())
    {
    vtkIdType nextVertex = dfsIterator->Next();
    cout << "Next vertex: " << nextVertex << endl;
    }
}

void BFS(vtkSmartPointer<vtkTree> tree)
{
  //this should produce 0, 1, 2, 3
  
  cout << "BFS" << endl << "----------" << endl;
  
  vtkIdType root = tree->GetRoot();
  
  vtkSmartPointer<vtkTreeBFSIterator> bfsIterator = 
      vtkSmartPointer<vtkTreeBFSIterator>::New();
  bfsIterator->SetStartVertex(root);
  bfsIterator->SetTree(tree);
 
  //traverse the tree in a depth first fashion
  while(bfsIterator->HasNext())
    {
    vtkIdType nextVertex = bfsIterator->Next();
    cout << "Next vertex: " << nextVertex << endl;
    }
  
}