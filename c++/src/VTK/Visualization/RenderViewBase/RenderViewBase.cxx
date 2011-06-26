#include <vtkRenderViewBase.h>
#include <vtkSphereSource.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>

int main(int, char*[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource =
    vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();

  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(sphereSource->GetOutputPort());

  vtkSmartPointer<vtkActor> actor =
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  vtkSmartPointer<vtkRenderViewBase> renderViewBase =
    vtkSmartPointer<vtkRenderViewBase>::New();

  renderViewBase->GetRenderer()->AddActor(actor);
  renderViewBase->SetInteractionMode(vtkRenderViewBase::INTERACTION_MODE_3D);
  renderViewBase->Update();

  renderViewBase->ResetCamera();
  renderViewBase->GetInteractor()->Start();

  return EXIT_SUCCESS;
}
