#include <vtkSmartPointer.h>
#include <vtkCommand.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkBoxRepresentation.h>
#include <vtkBoxWidget2.h>
#include <vtkUnstructuredGrid.h>
#include <vtkDataSetMapper.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPointSource.h>
#include <vtkSelectEnclosedPoints.h>
#include <vtkProperty.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkThreshold.h>
#include <vtkThresholdPoints.h>
#include <vtkPointData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkRendererCollection.h>
#include <vtkInteractorStyleTrackballCamera.h>

class vtkBoxCallback : public vtkCommand
{
  public:
    static vtkBoxCallback *New()
    {
      return new vtkBoxCallback;
    }
    vtkBoxCallback()
    {
       this->EnclosedActor = vtkSmartPointer<vtkActor>::New();
    }
 
    virtual void Execute(vtkObject *caller, unsigned long, void*)
    {
 
      vtkBoxWidget2 *boxWidget = 
          reinterpret_cast<vtkBoxWidget2*>(caller);
 
      //get the actual box coordinates/planes
      vtkSmartPointer<vtkPolyData> polydata = 
          vtkSmartPointer<vtkPolyData>::New();
      static_cast<vtkBoxRepresentation*>(boxWidget->GetRepresentation())->GetPolyData(polydata);
      
      //find enclosed points
      vtkSmartPointer<vtkSelectEnclosedPoints> enclosedFilter =
          vtkSmartPointer<vtkSelectEnclosedPoints>::New();
      enclosedFilter->SetInputConnection(this->Points->GetProducerPort());
      enclosedFilter->SetSurface(polydata);
      enclosedFilter->Update();
      
      //extract the enclosed points
      vtkSmartPointer<vtkThresholdPoints> threshold = 
            vtkSmartPointer<vtkThresholdPoints>::New();
      threshold->SetInputConnection(enclosedFilter->GetOutputPort());
      threshold->SetInputArrayToProcess(0,0,0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "SelectedPoints");
      threshold->ThresholdByUpper(0.9); //grab all the points that are marked "1" (inside the pyramid)
      threshold->Update();
      
      vtkSmartPointer<vtkDataSetMapper> enclosedMapper = 
          vtkSmartPointer<vtkDataSetMapper>::New();
      //enclosedMapper->SetInputConnection(glyphFilter->GetOutputPort());
      enclosedMapper->SetInputConnection(threshold->GetOutputPort());
      enclosedMapper->ScalarVisibilityOff();
      
      EnclosedActor->SetMapper(enclosedMapper);
      EnclosedActor->GetProperty()->SetPointSize(5);
      EnclosedActor->GetProperty()->SetColor(0.0, 1.0, 0.0); //(R,G,B) - should be bright green

      this->RenderWindow->GetRenderers()->GetFirstRenderer()->AddActor(EnclosedActor);
      this->RenderWindow->Render();
    }
 
    void SetPoints(vtkPolyData* points) {this->Points = points;}
    void SetRenderWindow(vtkRenderWindow* renderWindow) {this->RenderWindow = renderWindow;}
    
  private:
   vtkPolyData* Points;
   vtkRenderWindow* RenderWindow;
   vtkSmartPointer<vtkActor> EnclosedActor;
};

int main(int argc, char *argv[])
{
  //point cloud
  vtkSmartPointer<vtkPointSource> pointSource = 
      vtkSmartPointer<vtkPointSource>::New();
  pointSource->SetNumberOfPoints(500);
  pointSource->SetRadius(1);
  pointSource->Update();
  
  vtkSmartPointer<vtkDataSetMapper> pointMapper = 
      vtkSmartPointer<vtkDataSetMapper>::New();
  pointMapper->SetInputConnection(pointSource->GetOutputPort());

  vtkSmartPointer<vtkActor> pointActor = 
      vtkSmartPointer<vtkActor>::New();
  pointActor->SetMapper(pointMapper);
  
  //Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(pointActor);
  
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
  
  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = 
      vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
 renderWindowInteractor->SetInteractorStyle( style );
  
  vtkSmartPointer<vtkBoxWidget2> boxWidget = 
      vtkSmartPointer<vtkBoxWidget2>::New();
  boxWidget->SetInteractor(renderWindowInteractor);
  vtkSmartPointer<vtkBoxCallback> boxCallback = 
      vtkSmartPointer<vtkBoxCallback>::New();
  boxCallback->SetPoints(pointSource->GetOutput());
  boxCallback->SetRenderWindow(renderWindow);
  boxWidget->AddObserver(vtkCommand::InteractionEvent,boxCallback);
    
  //renderer->SetBackground(1,1,1);
  renderer->SetBackground(0,0,0);
  renderWindow->Render();
  boxWidget->On();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
