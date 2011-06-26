#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkDelaunay3D.h>

void ProgressFunction(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData);

int main(int, char *[])
{ 
  vtkSmartPointer<vtkSphereSource> sphereSource =
    vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  
  
  vtkSmartPointer<vtkCallbackCommand> progressCallback = 
    vtkSmartPointer<vtkCallbackCommand>::New();
  progressCallback->SetCallback(ProgressFunction);
  
  
  vtkSmartPointer<vtkDelaunay3D> delaunay = 
    vtkSmartPointer<vtkDelaunay3D>::New();
  delaunay->SetInputConnection(sphereSource->GetOutputPort());
  delaunay->AddObserver(vtkCommand::ProgressEvent, progressCallback);
  delaunay->Update();
  
  return EXIT_SUCCESS;
}

void ProgressFunction(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
{
  vtkDelaunay3D* filter = static_cast<vtkDelaunay3D*>(caller);
  std::cout << "Progress: " << filter->GetProgress() << std::endl;
}