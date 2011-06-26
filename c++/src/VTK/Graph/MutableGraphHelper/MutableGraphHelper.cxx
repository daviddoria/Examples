#include <vtkSmartPointer.h>
#include <vtkMutableDirectedGraph.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkMutableGraphHelper.h>
#include <vtkGraphLayoutView.h>
#include <vtkRenderWindowInteractor.h>

int main(int, char*[])
{

  vtkSmartPointer<vtkMutableGraphHelper> graphHelper =
      vtkSmartPointer<vtkMutableGraphHelper>::New();

  vtkIdType v0 = graphHelper->AddVertex();
  vtkIdType v1 = graphHelper->AddVertex();

  graphHelper->AddEdge(v0, v1);

  vtkSmartPointer<vtkGraphLayoutView> treeLayoutView =
      vtkSmartPointer<vtkGraphLayoutView>::New();
  treeLayoutView->AddRepresentationFromInput(graphHelper->GetGraph());
  treeLayoutView->SetLayoutStrategyToTree();
  treeLayoutView->ResetCamera();
  treeLayoutView->Render();
  treeLayoutView->GetInteractor()->Start();

  return EXIT_SUCCESS;
}
