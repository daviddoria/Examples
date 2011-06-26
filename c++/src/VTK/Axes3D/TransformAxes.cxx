#include <vtkPolyDataMapper.h>
#include <vtkTransform.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextProperty.h>
#include <vtkSmartPointer.h>
#include <vtkTextActor.h>

int main (int argc, char *argv[])
{
  // a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
  
  vtkSmartPointer<vtkAxesActor> originAxes = 
      vtkSmartPointer<vtkAxesActor>::New();
  originAxes->SetXAxisLabelText("Xo");
  originAxes->SetYAxisLabelText("Yo");
  originAxes->SetZAxisLabelText("Zo");
  
  renderer->AddActor(originAxes);
  
  
  vtkSmartPointer<vtkAxesActor> axes = 
      vtkSmartPointer<vtkAxesActor>::New();
  axes->GetXAxisCaptionActor2D ()->GetTextActor()->SetTextScaleModeToNone();
  
  vtkSmartPointer<vtkTransform> transform =
      vtkSmartPointer<vtkTransform>::New();
  //transform->Translate(4.0, 0.0, 0.0);
  transform->RotateY(45);

  // The axes are positioned with a user transform
  axes->SetUserTransform(transform);
    
  //axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(1,0,0);
  axes->SetXAxisLabelText("x");
  axes->SetYAxisLabelText("y");
  axes->SetZAxisLabelText("z");
  //axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize (20);
  //axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->ShadowOff();
  
  renderer->AddActor(axes);
  
  //render and interact
  renderer->ResetCamera();
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}