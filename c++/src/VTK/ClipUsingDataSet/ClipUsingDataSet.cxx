#include <vtkSmartPointer.h>
#include <vtkCellData.h>
#include <vtkIdFilter.h>
#include <vtkIdTypeArray.h>
#include <vtkProperty.h>
#include <vtkDelaunay3D.h>
#include <vtkElevationFilter.h>
#include <vtkClipPolyData.h>
#include <vtkImplicitDataSet.h>
#include <vtkSphereSource.h>
#include <vtkCubeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

int main(int argc, char **argv)
{
  
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetCenter(.25, 0, 0);
  //unsigned int res = 100;
  unsigned int res = 10;
  sphereSource->SetThetaResolution(res);
  sphereSource->SetPhiResolution(res);
  sphereSource->Update();
  
  cout << "The sphere has " << sphereSource->GetOutput()->GetNumberOfPoints() << " points." << endl;
  cout << "The sphere has " << sphereSource->GetOutput()->GetNumberOfCells() << " cells." << endl;
  
  //add ids to the points and cells of the sphere
  vtkSmartPointer<vtkIdFilter> idFilter = 
      vtkSmartPointer<vtkIdFilter>::New();
  idFilter->SetInputConnection(sphereSource->GetOutputPort());
  idFilter->Update();
  
  vtkSmartPointer<vtkCubeSource> cubeSource = 
      vtkSmartPointer<vtkCubeSource>::New();
  cubeSource->Update();
  
  // Create 3D cells so vtkImplicitDataSet evaluates inside vs outside correctly
  vtkSmartPointer<vtkDelaunay3D> delaunayFilter =
      vtkSmartPointer<vtkDelaunay3D>::New();
  delaunayFilter->SetInput(cubeSource->GetOutput());
  delaunayFilter->BoundingTriangulationOff();
  delaunayFilter->Update();
  
  // vtkImplicitDataSet needs some scalars to interpolate to find inside/outside
  vtkSmartPointer<vtkElevationFilter> elevationFilter =
      vtkSmartPointer<vtkElevationFilter>::New();
  elevationFilter->SetInputConnection(delaunayFilter->GetOutputPort());
  elevationFilter->Update();
    
  vtkSmartPointer<vtkImplicitDataSet> implicitCube = 
      vtkSmartPointer<vtkImplicitDataSet>::New();
  implicitCube->SetDataSet(elevationFilter->GetOutput());
  
  vtkSmartPointer<vtkClipPolyData> clipper =
      vtkSmartPointer<vtkClipPolyData>::New();
  clipper->SetClipFunction(implicitCube);
  //clipper->SetInputConnection(sphereSource->GetOutputPort());
  clipper->SetInputConnection(idFilter->GetOutputPort());
  clipper->Update();
  
  //get the clipped cell ids
  vtkPolyData* clipped = clipper->GetOutput();
  vtkIdTypeArray* ids = vtkIdTypeArray::SafeDownCast(clipped->GetCellData()->GetArray("vtkIdFilter_Ids"));
  for(vtkIdType i = 0; i < ids->GetNumberOfTuples(); i++)
    {
    cout << "id " << i << " : " << ids->GetValue(i) << endl;
    }
  
  //Create a mapper and actor for clipped sphere
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(clipper->GetOutputPort());
  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
 
    //Create a mapper and actor for cube
  vtkSmartPointer<vtkPolyDataMapper> cubeMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  cubeMapper->SetInputConnection(cubeSource->GetOutputPort());
  vtkSmartPointer<vtkActor> cubeActor = 
      vtkSmartPointer<vtkActor>::New();
  cubeActor->SetMapper(cubeMapper);
  cubeActor->GetProperty()->SetOpacity(0.5);
  
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
  renderer->AddActor(cubeActor);
  renderer->SetBackground(1,1,1); // Background color white
 
  //Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
