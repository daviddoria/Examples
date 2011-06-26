#include <vtkSphereSource.h>
#include <vtkProperty.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

#include <vtksys/ios/sstream>
#include <vtkstd/vector>

int main(int argc, char *argv[])
{
  vtkstd::vector<vtkSmartPointer<vtkRenderWindowInteractor> > interactors;
  
  for(unsigned int i = 0; i < 4; i++)
    {
    vtkSmartPointer<vtkRenderWindow> renderWindow = 
        vtkSmartPointer<vtkRenderWindow>::New();
    
    vtkSmartPointer<vtkRenderer> renderer =
        vtkSmartPointer<vtkRenderer>::New();
    
    renderWindow->AddRenderer(renderer);
    
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
        vtkSmartPointer<vtkRenderWindowInteractor>::New();
    
    interactors.push_back(renderWindowInteractor);
    
    renderWindowInteractor->SetRenderWindow(renderWindow);
    renderWindow->Render();
    std::stringstream ss;
    ss << "Window " << i;
    renderWindow->SetWindowName(ss.str().c_str());
    
    //create a sphere
    vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetCenter(0.0, 0.0, 0.0);
    sphereSource->SetRadius(5.0);
    sphereSource->Update();
    vtkPolyData* sphere = sphereSource->GetOutput();
      
    //Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = 
        vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(sphereSource->GetOutputPort());
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    renderer->AddActor(actor);
    renderer->ResetCamera();
    }
    
    interactors[3]->Start();
    
  return 0;
}