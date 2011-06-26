#include <vtkSmartPointer.h>
#include <vtkGlyph3D.h>
#include <vtkCubeSource.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPointPicker.h>
#include <vtkPointData.h>
#include <vtkIdTypeArray.h>
#include <vtkRendererCollection.h>
#include <vtkProperty.h>
#include <vtkPlanes.h>
#include <vtkObjectFactory.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkPointSource.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkExtractGeometry.h>
#include <vtkDataSetMapper.h>
#include <vtkUnstructuredGrid.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkIdFilter.h>

// Define interaction style
class InteractorStyle : public vtkInteractorStyleTrackballActor
{
  public:
    static InteractorStyle* New();
    vtkTypeMacro(InteractorStyle,vtkInteractorStyleTrackballActor);

    InteractorStyle()
    {
      PointPicker = vtkSmartPointer<vtkPointPicker>::New();
      this->Move = false;

      this->MovePoints = vtkSmartPointer<vtkPoints>::New();
      this->MovePoints->InsertNextPoint(0,0,0);

      this->MovePolyData = vtkSmartPointer<vtkPolyData>::New();
      this->MovePolyData->SetPoints(this->MovePoints);

      this->GlyphSource = vtkSmartPointer<vtkCubeSource>::New();

      this->MoveGlyphFilter = vtkSmartPointer<vtkGlyph3D>::New();
      this->MoveGlyphFilter->SetSource(this->GlyphSource->GetOutput());
      this->MoveGlyphFilter->SetInput(this->MovePolyData);
      this->MoveGlyphFilter->Update();

      this->MoveMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      this->MoveMapper->SetInputConnection(this->MoveGlyphFilter->GetOutputPort());

      this->MoveActor = vtkSmartPointer<vtkActor>::New();
      this->MoveActor->SetMapper(this->MoveMapper);
      this->MoveActor->GetProperty()->SetPointSize(10);
      this->MoveActor->VisibilityOff();
    }

    void OnMiddleButtonUp()
    {
      // Forward events
      vtkInteractorStyleTrackballActor::OnMiddleButtonUp();

      if(this->Move)
    {
    // Hide the cube
    this->Move = false;
    this->MoveActor->VisibilityOff();

    double p[3];
    this->MoveActor->GetPosition(p);
    std::cout << "p: " << p[0] << " " << p[1] << " " << p[2] << std::endl;

    // Set the underlying point to the new position
    this->Data->GetPoints()->SetPoint(this->SelectedPoint, p);
    this->Data->Modified();

    // Refresh
    this->GetCurrentRenderer()->Render();
    this->GetCurrentRenderer()->GetRenderWindow()->Render();
    }
    }

    void OnMouseMove()
    {
      if(!this->Move)
        {
        return;
        }

      vtkInteractorStyleTrackballActor::OnMouseMove();
      double p[3];
      this->MoveActor->GetPosition(p);
      std::cout << "Current position: " << p[0] << " " << p[1] << " " << p[2] << std::endl;
    }

    void OnMiddleButtonDown()
    {
      // Forward events
      vtkInteractorStyleTrackballActor::OnMiddleButtonDown();

      // Get the selected point
      int x = this->Interactor->GetEventPosition()[0];
      int y = this->Interactor->GetEventPosition()[1];
      this-PointPicker->Pick(this->Interactor->GetEventPosition()[0],
                 this->Interactor->GetEventPosition()[1],
                 0,  // always zero.
                 this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());

      if(this->PointPicker->GetPointId() >= 0)
    {
    // Show the cube
    this->Move = true;
    this->MoveActor->VisibilityOn();
    std::cout << "Picked point: " << this->PointPicker->GetPointId() << std::endl;

    // Place the cube at the selected point
    this->SelectedPoint = this->PointPicker->GetPointId();
    double p[3];
    this->Data->GetPoint(this->SelectedPoint, p);
    std::cout << "Original coordinates: " << p[0] << " " << p[1] << " " << p[2] << std::endl;
    this->MovePolyData->GetPoints()->SetPoint(0,p);
    this->MovePolyData->Modified();

    // Trick the interactor into interacting with our cube
    this->InteractionProp = this->MoveActor;
    this->GetCurrentRenderer()->AddActor(this->MoveActor);

    // Refresh
    this->GetCurrentRenderer()->Render();
    this->GetCurrentRenderer()->GetRenderWindow()->Render();
    }
    }

    vtkPolyData* Data;

  protected:
    vtkSmartPointer<vtkPointPicker> PointPicker;
    bool Move;

    vtkSmartPointer<vtkPolyDataMapper> MoveMapper;
    vtkSmartPointer<vtkGlyph3D> MoveGlyphFilter;
    vtkSmartPointer<vtkCubeSource> GlyphSource;
    vtkSmartPointer<vtkActor> MoveActor;
    vtkSmartPointer<vtkPolyData> MovePolyData;
    vtkSmartPointer<vtkPoints> MovePoints;

    vtkIdType SelectedPoint;
};
vtkStandardNewMacro(InteractorStyle);

int main (int, char *[])
{
  // Create 3 points
  vtkSmartPointer<vtkPoints> points =
    vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(0,0,0);
  points->InsertNextPoint(1,0,0);
  points->InsertNextPoint(2,0,0);

  vtkSmartPointer<vtkPolyData> polydata =
    vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);

  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
    vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputConnection(polydata->GetProducerPort());
  glyphFilter->Update();

  vtkPolyData* input = glyphFilter->GetOutput();

  // Visualize
  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(input->GetProducerPort());

  vtkSmartPointer<vtkActor> actor =
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetPointSize(10);

  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderer->AddActor(actor);
  //renderer->SetBackground(1,1,1); // Background color white

  renderWindow->Render();

  vtkSmartPointer<InteractorStyle> style =
    vtkSmartPointer<InteractorStyle>::New();
  style->Data = input;
  renderWindowInteractor->SetInteractorStyle(style);

  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}