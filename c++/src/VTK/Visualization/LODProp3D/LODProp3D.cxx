#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkLODProp3D.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkCallbackCommand.h>

void RefreshCallback( vtkObject*
                      caller, long unsigned int eventId, void* clientData, void* callData )
{
  vtkSmartPointer<vtkLODProp3D> lodProp = 
    static_cast<vtkLODProp3D*>(clientData);
  cout << "Last rendered LOD: " << lodProp->GetLastRenderedLODID() << endl;
}

int main (int argc, char *argv[])
{
  //high res sphere
  vtkSmartPointer<vtkSphereSource> highResSphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  int res = 100;
  highResSphereSource->SetThetaResolution(res);
  highResSphereSource->SetPhiResolution(res);
  highResSphereSource->Update();
  
  vtkSmartPointer<vtkPolyDataMapper> highResMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  highResMapper->SetInputConnection(highResSphereSource->GetOutputPort());
  
  //low res sphere
  vtkSmartPointer<vtkSphereSource> lowResSphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
    
  vtkSmartPointer<vtkPolyDataMapper> lowResMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  lowResMapper->SetInputConnection(lowResSphereSource->GetOutputPort());
  
  vtkSmartPointer<vtkLODProp3D> prop = 
      vtkSmartPointer<vtkLODProp3D>::New();
  prop->AddLOD(lowResMapper, 0.0);
  prop->AddLOD(highResMapper, 0.0);
  
  cout << "There are " << prop->GetNumberOfLODs() << " LODs" << endl;
    
  // a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  //prop->SetAllocatedRenderTime(1e-6,renderer);
  prop->SetAllocatedRenderTime(1e-10,renderer);
      
  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // add the actors to the scene
  renderer->AddActor(prop);
  
  vtkSmartPointer<vtkCallbackCommand> refreshCallback =
    vtkSmartPointer<vtkCallbackCommand>::New();
  refreshCallback->SetCallback (RefreshCallback);
  refreshCallback->SetClientData(prop);

  renderWindow->AddObserver(vtkCommand::ModifiedEvent,refreshCallback);
  
  renderWindow->Render();

  // begin mouse interaction
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
