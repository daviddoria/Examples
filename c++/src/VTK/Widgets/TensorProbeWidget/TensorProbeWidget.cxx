#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkTensorProbeWidget.h>

int main(int argc, char *argv[])
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

  vtkSmartPointer<vtkTensorProbeWidget> tensorProbeWidget = 
      vtkSmartPointer<vtkTensorProbeWidget>::New();
  tensorProbeWidget->SetInteractor(renderWindowInteractor);
  
  renderWindowInteractor->Initialize();
  renderWindow->Render();
  tensorProbeWidget->On();
  renderer->ResetCamera();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
