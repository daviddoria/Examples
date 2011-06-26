#include <vtkRenderView.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkRenderedSurfaceRepresentation.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>

int main (int, char *[])
{

  vtkSmartPointer<vtkSphereSource> sphereSource =
    vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetCenter(0.0, 0.0, 0.0);
  sphereSource->SetRadius(5.0);
  sphereSource->Update();

  vtkPolyData* sphere = sphereSource->GetOutput();

  vtkSmartPointer<vtkRenderedSurfaceRepresentation> renderedSurfaceRepresentation =
    vtkSmartPointer<vtkRenderedSurfaceRepresentation>::New();
  renderedSurfaceRepresentation->SetInput(sphere);

  vtkSmartPointer<vtkRenderView> renderView =
    vtkSmartPointer<vtkRenderView>::New();
  renderView->AddRepresentation(renderedSurfaceRepresentation);
  renderView->Update();

  renderView->ResetCamera();
  renderView->GetInteractor()->Start();

  return EXIT_SUCCESS;
}
