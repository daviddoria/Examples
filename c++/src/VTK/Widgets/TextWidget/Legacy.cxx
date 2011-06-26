#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

#include <vtkTextWidget.h>

int main(int, char*[])
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

  vtkSmartPointer<vtkTextWidget> textWidget =
      vtkSmartPointer<vtkTextWidget>::New();
  textWidget->SetInteractor(renderWindowInteractor);

  renderWindowInteractor->Initialize();
  renderWindow->Render();
  textWidget->On();
  renderer->ResetCamera();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
