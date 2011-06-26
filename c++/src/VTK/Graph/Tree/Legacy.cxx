#include <vtkSmartPointer.h>
#include <vtkTree.h>
#include <vtkDoubleArray.h>
#include <vtkDataSetAttributes.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkBoostPrimMinimumSpanningTree.h>
#include <vtkGraphLayoutView.h>
#include <vtkRenderWindowInteractor.h>

int main ( int argc, char *argv[] )
{
  vtkSmartPointer<vtkMutableUndirectedGraph> g = 
      vtkSmartPointer<vtkMutableUndirectedGraph>::New();

  vtkIdType V1 = g->AddVertex();
  vtkIdType V2 = g->AddVertex();
  vtkIdType V3 = g->AddVertex();
  vtkIdType V4 = g->AddVertex();

  g->AddEdge ( V1, V2 );
  g->AddEdge ( V1, V3 );
  g->AddEdge ( V2, V3 );
  g->AddEdge ( V2, V4 );
  g->AddEdge ( V3, V4 );
  
  //create the edge weight array
  vtkSmartPointer<vtkDoubleArray> weights = 
      vtkSmartPointer<vtkDoubleArray>::New();
  weights->SetNumberOfComponents(1);
  weights->SetName("Weights");
  
  for(unsigned int i = 0; i < g->GetNumberOfEdges(); i++)
    {
    weights->InsertNextValue(1.0);
    }
  
  //add the edge weight array to the graph
  g->GetEdgeData()->AddArray(weights);
  
  vtkSmartPointer<vtkGraphLayoutView> graphLayoutView = 
      vtkSmartPointer<vtkGraphLayoutView>::New();
  graphLayoutView->AddRepresentationFromInput(g);
  graphLayoutView->SetLayoutStrategyToSimple2D();
  //graphLayoutView->SetLayoutStrategyToTree();
  graphLayoutView->ResetCamera();
  graphLayoutView->Render();
  graphLayoutView->GetInteractor()->Start();
  
  //find the minimum spanning tree on this graph, starting from the highest point
  vtkSmartPointer<vtkBoostPrimMinimumSpanningTree> minimumSpanningTreeFilter = 
      vtkSmartPointer<vtkBoostPrimMinimumSpanningTree>::New();
  minimumSpanningTreeFilter->SetOriginVertex(0);
  minimumSpanningTreeFilter->SetInput(g);
  minimumSpanningTreeFilter->SetEdgeWeightArrayName("Weights");
  minimumSpanningTreeFilter->Update();
  
  vtkSmartPointer<vtkGraphLayoutView> treeLayoutView = 
      vtkSmartPointer<vtkGraphLayoutView>::New();
  treeLayoutView->AddRepresentationFromInput(minimumSpanningTreeFilter->GetOutput());
  treeLayoutView->SetLayoutStrategyToTree();
  treeLayoutView->ResetCamera();
  treeLayoutView->Render();
  treeLayoutView->GetInteractor()->Start();
  
  return EXIT_SUCCESS;
}
