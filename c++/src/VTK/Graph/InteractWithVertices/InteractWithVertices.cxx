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


class CustomStrategy : public vtkSimple2DLayoutStrategy
{
  public:
    static CustomStrategy* New();
    vtkTypeMacro(CustomStrategy, vtkSimple2DLayoutStrategy);
    vtkGraph* GetGraph(){return this->Graph;}
};
vtkStandardNewMacro(CustomStrategy);

class CustomStyle : public vtkInteractorStyleTrackballActor
{
  public:
    static CustomStyle* New();
    vtkTypeMacro(CustomStyle,vtkInteractorStyleTrackballActor);

    CustomStyle()
    {
      this->Move = false;
      this->PointPicker = vtkSmartPointer<vtkPointPicker>::New();

      // Setup ghost glyph
      vtkSmartPointer<vtkPoints> points =
        vtkSmartPointer<vtkPoints>::New();
      points->InsertNextPoint(0,0,0);
      this->MovePolyData = vtkSmartPointer<vtkPolyData>::New();
      this->MovePolyData->SetPoints(points);
      this->MoveGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
      this->MoveGlyphFilter->SetInputConnection(this->MovePolyData->GetProducerPort());
      this->MoveGlyphFilter->Update();

      this->MoveMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      this->MoveMapper->SetInputConnection(this->MoveGlyphFilter->GetOutputPort());

      this->MoveActor = vtkSmartPointer<vtkActor>::New();
      this->MoveActor->SetMapper(this->MoveMapper);
      this->MoveActor->VisibilityOff();
      this->MoveActor->GetProperty()->SetPointSize(10);
      this->MoveActor->GetProperty()->SetColor(1,0,0);
    }

    void OnMiddleButtonUp()
    {
      this->EndPan();

      this->Move = false;
      this->MoveActor->VisibilityOff();

      //this->Graph->GetPoints()->SetPoint(this->SelectedPoint, this->MoveActor->GetPosition());

      this->Graph->GetPoints()->SetPoint(this->SelectedPoint, this->MoveActor->GetPosition());
      this->GraphLayoutView->UpdateLayout();
      //this->GraphLayoutView->SetRepresentationFromInput(this->Graph);
      this->Graph->Modified();
      this->GetCurrentRenderer()->Render();
      this->GetCurrentRenderer()->GetRenderWindow()->Render();

    }

    void OnMiddleButtonDown()
    {
      this->Graph = CustomStrategy::SafeDownCast(this->GraphLayoutView->GetLayoutStrategy())->GetGraph();
      std::cout << "this->Graph: " << this->Graph << std::endl;
/*
      std::cout << "Graph points: " << std::endl;
      for(vtkIdType i = 0; i < this->Graph->GetPoints()->GetNumberOfPoints(); i++)
        {
        double p[3];
        this->Graph->GetPoint(i, p);
        std::cout << "p: " << p[0] << " " << p[1] << " " << p[2] << std::endl;
        }
*/
      // Get the selected point
      int x = this->Interactor->GetEventPosition()[0];
      int y = this->Interactor->GetEventPosition()[1];
      this->FindPokedRenderer(x, y);

      this->PointPicker->Pick(this->Interactor->GetEventPosition()[0],
                 this->Interactor->GetEventPosition()[1],
                 0,  // always zero.
                 this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());

      if(this->PointPicker->GetPointId() >= 0)
        {
        this->StartPan();
        this->MoveActor->VisibilityOn();
        this->Move = true;
        this->SelectedPoint = this->PointPicker->GetPointId();

        double p[3];
        this->Graph->GetPoint(this->SelectedPoint, p);
        std::cout << "p: " << p[0] << " " << p[1] << " " << p[2] << std::endl;
        this->MoveActor->SetPosition(p);

        this->GetCurrentRenderer()->AddActor(this->MoveActor);
        this->InteractionProp = this->MoveActor;

        }
    }

  vtkGraph* Graph;
  vtkGraphLayoutView* GraphLayoutView;

private:
  vtkSmartPointer<vtkPointPicker> PointPicker;

  vtkPolyData* GlyphData;

  vtkSmartPointer<vtkPolyDataMapper> MoveMapper;
  vtkSmartPointer<vtkActor> MoveActor;
  vtkSmartPointer<vtkPolyData> MovePolyData;
  vtkSmartPointer<vtkVertexGlyphFilter> MoveGlyphFilter;


  bool Move;
  vtkIdType SelectedPoint;
};
vtkStandardNewMacro(CustomStyle);

int main(int, char *[])
{
  vtkSmartPointer<vtkMutableUndirectedGraph> g =
    vtkSmartPointer<vtkMutableUndirectedGraph>::New();
  std::cout << "Original g: " << g << std::endl;
  vtkIdType v1 = g->AddVertex();
  vtkIdType v2 = g->AddVertex();

  g->AddEdge(v1, v2);
  g->AddEdge(v1, v2);

  vtkSmartPointer<CustomStrategy> strategy =
    vtkSmartPointer<CustomStrategy>::New();
  vtkSmartPointer<vtkGraphLayoutView> graphLayoutView =
    vtkSmartPointer<vtkGraphLayoutView>::New();
  graphLayoutView->SetRepresentationFromInput(g);

  vtkSmartPointer<CustomStyle> style =
    vtkSmartPointer<CustomStyle>::New();
  style->GraphLayoutView = graphLayoutView;
  graphLayoutView->GetInteractor()->SetInteractorStyle(style);
  graphLayoutView->SetLayoutStrategy(strategy);
  graphLayoutView->ResetCamera();
  graphLayoutView->Render();
  graphLayoutView->GetInteractor()->Start();

  return EXIT_SUCCESS;
}
