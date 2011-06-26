#include <vtkSmartPointer.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkGraphWriter.h>
#include <vtkGraphLayoutView.h>
#include <vtkRenderWindowInteractor.h>

int main ( int argc, char *argv[] )
{
  vtkSmartPointer<vtkMutableUndirectedGraph> g = 
      vtkSmartPointer<vtkMutableUndirectedGraph>::New();

  vtkIdType v1 = g->AddVertex();
  vtkIdType v2 = g->AddVertex();
  vtkIdType v3 = g->AddVertex();

  g->AddEdge ( v1, v2 );
  g->AddEdge ( v1, v3 );
  
  vtkSmartPointer<vtkGraphLayoutView> graphLayoutView = 
      vtkSmartPointer<vtkGraphLayoutView>::New();
  graphLayoutView->AddRepresentationFromInput(g);
  graphLayoutView->SetLayoutStrategyToTree();
  graphLayoutView->ResetCamera();
  graphLayoutView->Render();
  
  graphLayoutView->GetInteractor()->Start();
  
  return EXIT_SUCCESS;
}
