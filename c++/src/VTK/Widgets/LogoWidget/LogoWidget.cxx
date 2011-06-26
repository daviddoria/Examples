#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkLogoWidget.h>
#include <vtkLogoRepresentation.h>
#include <vtkImageCanvasSource2D.h>

int main(int argc, char *argv[])
{
 
  // a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  //renderer->AddActor(actor);
  
  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  //create an image
  vtkSmartPointer<vtkImageCanvasSource2D> drawing = 
    vtkSmartPointer<vtkImageCanvasSource2D>::New();
  drawing->SetScalarTypeToUnsignedChar();
  drawing->SetNumberOfScalarComponents(3);
  drawing->SetExtent(0,100,0,100,0,0);
  //clear the image
  drawing->SetDrawColor(0, 0, 0, 0);
  drawing->FillBox(0,100,0,100);
  //drawing->SetDrawColor(255.0, 0.0, 0.0, 1.0);
  drawing->SetDrawColor(255.0, 0.0, 0.0, 0);
  //drawing->DrawCircle(50, 50, 50);
  drawing->FillBox(0, 100, 0, 100);
  drawing->Update();

  vtkSmartPointer<vtkLogoRepresentation> logoRepresentation = 
    vtkSmartPointer<vtkLogoRepresentation>::New();
  logoRepresentation->SetImage(drawing->GetOutput());
  
  vtkSmartPointer<vtkLogoWidget> logoWidget = 
      vtkSmartPointer<vtkLogoWidget>::New();
  logoWidget->SetInteractor(renderWindowInteractor);
  logoWidget->SetRepresentation(logoRepresentation);
  
  renderWindow->Render();
  renderWindowInteractor->Initialize();
  renderWindow->Render();
  logoWidget->On();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
