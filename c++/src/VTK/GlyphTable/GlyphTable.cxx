#include <vtkSmartPointer.h>
#include <vtkPointData.h>
#include <vtkCubeSource.h>
#include <vtkConeSource.h>
#include <vtkSphereSource.h>
#include <vtkGlyph3D.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkFloatArray.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkSphereSource> bigSphereSource = 
    vtkSmartPointer<vtkSphereSource>::New();
  bigSphereSource->Update();
  
  vtkSmartPointer<vtkFloatArray> height = 
    vtkSmartPointer<vtkFloatArray>::New();
  height->SetNumberOfComponents(1);
  height->SetName("Height");
  
  for(vtkIdType i = 0; i < bigSphereSource->GetOutput()->GetNumberOfPoints(); i++)
    {
    double p[3];
    bigSphereSource->GetOutput()->GetPoint(i, p);
    height->InsertNextValue(p[2]);
    }
  
  vtkSmartPointer<vtkPolyData> input = 
    vtkSmartPointer<vtkPolyData>::New();
  input->ShallowCopy(bigSphereSource->GetOutput());
  input->GetPointData()->AddArray(height);
  
  vtkSmartPointer<vtkConeSource> coneSource =
    vtkSmartPointer<vtkConeSource>::New();
  coneSource->SetRadius(.1);
  coneSource->SetHeight(.1);
  coneSource->Update();
  
  vtkSmartPointer<vtkCubeSource> cubeSource =
    vtkSmartPointer<vtkCubeSource>::New();
  cubeSource->SetXLength(.1);
  cubeSource->SetYLength(.1);
  cubeSource->SetZLength(.1);
  cubeSource->Update();
  
  vtkSmartPointer<vtkGlyph3D> glyph3D = 
    vtkSmartPointer<vtkGlyph3D>::New();
  glyph3D->SetIndexModeToScalar();
  glyph3D->SetInputArrayToProcess(0,0,0,0,"Height");
  glyph3D->SetSource(0, cubeSource->GetOutput());
  glyph3D->SetSource(1, coneSource->GetOutput());
  glyph3D->SetInput(input);
  glyph3D->SetRange(-1, 1);
  glyph3D->Update();

    //Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(glyph3D->GetOutputPort());
 
  vtkSmartPointer<vtkActor> actor = 
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
 
  //Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer = 
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
 
  //Add the actor to the scene
  renderer->AddActor(actor);
  renderer->SetBackground(.3, .6, .3); // Background color green
 
  //Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
