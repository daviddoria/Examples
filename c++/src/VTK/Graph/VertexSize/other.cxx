#include <vtkSmartPointer.h>
#include <vtkRenderedGraphRepresentation.h>
#include <vtkObjectFactory.h>
#include <vtkFloatArray.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkGraphWriter.h>
#include <vtkGraphLayoutView.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkDataSetAttributes.h>
#include <vtkGraphToGlyphs.h>

class CustomRepresentation : public vtkRenderedGraphRepresentation
{
 public:
    static CustomRepresentation* New();
    vtkTypeMacro(CustomRepresentation, vtkRenderedGraphRepresentation);

    void SetVertexSize(int vertexSize)
    {
      this->VertexGlyph->SetScreenSize(vertexSize);
      this->VertexGlyph->Modified();
      this->VertexGlyph->Update();
    }
};
vtkStandardNewMacro(CustomRepresentation);

int main(int, char *[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> g =
    vtkSmartPointer<vtkMutableUndirectedGraph>::New();

  vtkIdType v1 = g->AddVertex();
  vtkIdType v2 = g->AddVertex();

  g->AddEdge(v1, v2);
  g->AddEdge(v1, v2);

  vtkSmartPointer<CustomRepresentation> representation =
    vtkSmartPointer<CustomRepresentation>::New();
  representation->SetInputConnection(g->GetProducerPort());
  representation->SetVertexSize(100);
  representation->Update();

  vtkSmartPointer<vtkGraphLayoutView> layoutView =
    vtkSmartPointer<vtkGraphLayoutView>::New();
  layoutView->AddRepresentation(representation);
  layoutView->ResetCamera();
  layoutView->Render();
  layoutView->GetInteractor()->Start();

  return EXIT_SUCCESS;
}
