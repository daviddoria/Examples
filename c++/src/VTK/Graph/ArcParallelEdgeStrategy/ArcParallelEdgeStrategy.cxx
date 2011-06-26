#include <vtkSmartPointer.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkGraphWriter.h>
#include <vtkGraphLayoutView.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkGraphLayoutStrategy.h>
#include <vtkArcParallelEdgeStrategy.h>
#include <vtkRenderedGraphRepresentation.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> g =
    vtkSmartPointer<vtkMutableUndirectedGraph>::New();

  vtkIdType v1 = g->AddVertex();
  vtkIdType v2 = g->AddVertex();

  g->AddEdge ( v1, v2 );
  g->AddEdge ( v1, v2 );

  vtkSmartPointer<vtkGraphLayoutView> graphLayoutView =
    vtkSmartPointer<vtkGraphLayoutView>::New();
  graphLayoutView->AddRepresentationFromInput(g);

  vtkMutableUndirectedGraph* g2 =
    vtkMutableUndirectedGraph::SafeDownCast(vtkRenderedGraphRepresentation::SafeDownCast(graphLayoutView->GetRepresentation())->GetInput());
  std::cout << "g2 verts: " << g2->GetNumberOfVertices() << std::endl;

  graphLayoutView->ResetCamera();
  graphLayoutView->Render();
  graphLayoutView->SetEdgeLayoutStrategyToArcParallel();

  graphLayoutView->GetInteractor()->Start();

  return EXIT_SUCCESS;
}
