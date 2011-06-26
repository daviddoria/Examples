#include <vtkSmartPointer.h>
#include <vtkFloatArray.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkGraphToGlyphs.h>
#include <vtkGraphWriter.h>
#include <vtkGraphLayoutView.h>
#include <vtkRenderedGraphRepresentation.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkDataSetAttributes.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> g =
    vtkSmartPointer<vtkMutableUndirectedGraph>::New();

  vtkIdType v1 = g->AddVertex();
  vtkIdType v2 = g->AddVertex();

  g->AddEdge(v1, v2);
  g->AddEdge(v1, v2);

  vtkSmartPointer<vtkFloatArray> scales =
    vtkSmartPointer<vtkFloatArray>::New();
  scales->SetNumberOfComponents(1);
  scales->SetName("Scales");
  scales->InsertNextValue(1.0);
  scales->InsertNextValue(4.0);

  g->GetVertexData()->AddArray(scales);

  vtkSmartPointer<vtkGraphLayoutView> graphLayoutView =
    vtkSmartPointer<vtkGraphLayoutView>::New();
  graphLayoutView->AddRepresentationFromInput(g);
  graphLayoutView->ScaledGlyphsOn();
  graphLayoutView->SetScalingArrayName("Scales");
  vtkRenderedGraphRepresentation::SafeDownCast(graphLayoutView->GetRepresentation())->SetGlyphType(vtkGraphToGlyphs::CIRCLE);

  graphLayoutView->ResetCamera();
  graphLayoutView->Render();
  graphLayoutView->GetInteractor()->Start();

  return EXIT_SUCCESS;
}