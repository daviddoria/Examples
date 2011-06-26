#include <vtkMutableUndirectedGraph.h>
#include <vtkDoubleArray.h>
#include <vtkIntArray.h>
#include <vtkDataSetAttributes.h>
#include <vtkSmartPointer.h>

#include <vtkGraphLayoutView.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCircularLayoutStrategy.h>

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
  
  //create the edge weight array
  vtkSmartPointer<vtkIntArray> vertexIDs = 
      vtkSmartPointer<vtkIntArray>::New();
  vertexIDs->SetNumberOfComponents(1);
  vertexIDs->SetName("VertexIDs");
  
  //set the edge weights
  vertexIDs->InsertNextValue(0);
  vertexIDs->InsertNextValue(1);
  vertexIDs->InsertNextValue(2);
  
  //add the edge weight array to the graph
  g->GetEdgeData()->AddArray(weights);
  g->GetVertexData()->AddArray(vertexIDs);
  
  vtkSmartPointer<vtkCircularLayoutStrategy> circularLayoutStrategy = 
      vtkSmartPointer<vtkCircularLayoutStrategy>::New();
  
  vtkSmartPointer<vtkGraphLayoutView> graphLayoutView = 
      vtkSmartPointer<vtkGraphLayoutView>::New();
  graphLayoutView->AddRepresentationFromInput(g);
  
  graphLayoutView->SetLayoutStrategy(circularLayoutStrategy);
  graphLayoutView->SetVertexLabelVisibility(true);
  graphLayoutView->SetEdgeLabelVisibility(true);
  graphLayoutView->SetEdgeLabelArrayName("Weights"); //default is "labels"
  graphLayoutView->SetVertexLabelArrayName("VertexIDs"); //default is "labels"
  graphLayoutView->ResetCamera();
  graphLayoutView->Render();
  graphLayoutView->GetInteractor()->Start();
  
  return EXIT_SUCCESS;
}
