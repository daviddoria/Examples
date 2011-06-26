#include <vtkSmartPointer.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkGraphWriter.h>
#include <vtkGraphLayoutView.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkGraphLayoutStrategy.h>
#include <vtkSimple2DLayoutStrategy.h>
#include <vtkRenderedGraphRepresentation.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkObjectFactory.h>
#include <vtkPointPicker.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkRenderer.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkInteractorStyleTrackballCamera.h>

class CustomStrategy : public vtkSimple2DLayoutStrategy
{
  public:
    static CustomStrategy* New();
    vtkTypeMacro(CustomStrategy, vtkSimple2DLayoutStrategy);
    vtkGraph* GetGraph(){return this->Graph;}
};
vtkStandardNewMacro(CustomStrategy);

class CustomStyle : public vtkInteractorStyleTrackballCamera
{
  public:
    static CustomStyle* New();
    vtkTypeMacro(CustomStyle, vtkInteractorStyleTrackballCamera);

    virtual void OnLeftButtonDown()
    {
      std::cout << "Pressed left mouse button." << std::endl;

      vtkGraph* gWithPoints = CustomStrategy::SafeDownCast(this->graphLayoutView->GetLayoutStrategy())->GetGraph();

      std::cout << gWithPoints->GetClassName() << std::endl;
      std::cout << gWithPoints << std::endl;

      std::cout << "Graph points: " << std::endl;
      for(vtkIdType i = 0; i < gWithPoints->GetPoints()->GetNumberOfPoints(); i++)
        {
        double p[3];
        gWithPoints->GetPoint(i, p);
        std::cout << "p: " << p[0] << " " << p[1] << " " << p[2] << std::endl;
        }

    }

vtkGraphLayoutView* graphLayoutView;
};

vtkStandardNewMacro(CustomStyle);

int main(int, char *[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> g =
    vtkSmartPointer<vtkMutableUndirectedGraph>::New();

  vtkIdType v1 = g->AddVertex();
  vtkIdType v2 = g->AddVertex();

  g->AddEdge(v1, v2);
  g->AddEdge(v1, v2);

  std::cout << "g: " << g << std::endl;


  vtkSmartPointer<CustomStrategy> strategy =
    vtkSmartPointer<CustomStrategy>::New();

  vtkSmartPointer<vtkGraphLayoutView> graphLayoutView =
    vtkSmartPointer<vtkGraphLayoutView>::New();
  graphLayoutView->AddRepresentationFromInput(g);
  graphLayoutView->SetLayoutStrategy(strategy);
  graphLayoutView->ResetCamera();
  graphLayoutView->Render();

  vtkSmartPointer<CustomStyle> style =
    vtkSmartPointer<CustomStyle>::New();
  graphLayoutView->GetInteractor()->SetInteractorStyle( style );
  style->graphLayoutView = graphLayoutView;
  graphLayoutView->GetInteractor()->Start();

  return EXIT_SUCCESS;
}
