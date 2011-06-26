#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkContourWidget.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

int main(int argc, char *[])
{
  // Create a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  // Create an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Create a contour widget
  vtkSmartPointer<vtkContourWidget> contourWidget =
      vtkSmartPointer<vtkContourWidget>::New();
  contourWidget->SetInteractor(renderWindowInteractor);
  contourWidget->CreateDefaultRepresentation();

  renderWindow->Render();
  renderWindowInteractor->Initialize();
  renderWindow->Render();
  contourWidget->On();

  // begin mouse interaction
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
