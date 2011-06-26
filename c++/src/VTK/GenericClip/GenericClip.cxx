#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkDataSetMapper.h>
#include <vtkSampleFunction.h>
#include <vtkContourFilter.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCellData.h>
#include <vtkIdFilter.h>
#include <vtkIdTypeArray.h>
#include <vtkProperty.h>
#include <vtkDelaunay3D.h>
#include <vtkElevationFilter.h>
#include <vtkGenericClip.h>
#include <vtkImplicitDataSet.h>
#include <vtkSphere.h>
#include <vtkPointSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

int main(int argc, char **argv)
{
  
  vtkSmartPointer<vtkPointSource> pointSource = 
      vtkSmartPointer<vtkPointSource>::New();
  pointSource->SetNumberOfPoints(100);
  pointSource->SetRadius(1.0);
  pointSource->Update();
  
  //add ids to the points and cells of the sphere
  vtkSmartPointer<vtkIdFilter> idFilter = 
      vtkSmartPointer<vtkIdFilter>::New();
  idFilter->SetInputConnection(pointSource->GetOutputPort());
  idFilter->Update();
  
  //create a sphere to clip with (default radius is 0.5)
  vtkSmartPointer<vtkSphere> sphere = 
      vtkSmartPointer<vtkSphere>::New();
    
  vtkSmartPointer<vtkGenericClip> clipper =
      vtkSmartPointer<vtkGenericClip>::New();
  clipper->SetClipFunction(sphere);
  clipper->SetInputConnection(idFilter->GetOutputPort());
  clipper->Update();
  
  //get the clipped cell ids
  vtkUnstructuredGrid* clipped = clipper->GetOutput();
  vtkIdTypeArray* ids = vtkIdTypeArray::SafeDownCast(clipped->GetCellData()->GetArray("vtkIdFilter_Ids"));
  for(vtkIdType i = 0; i < ids->GetNumberOfTuples(); i++)
    {
    cout << "id " << i << " : " << ids->GetValue(i) << endl;
    }
  
  //Create a mapper and actor for clipped points
  vtkSmartPointer<vtkDataSetMapper> mapper = 
      vtkSmartPointer<vtkDataSetMapper>::New();
  mapper->SetInputConnection(clipper->GetOutputPort());
  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
 
  /*
  //Create a mapper and actor for cube
  vtkSmartPointer<vtkPolyDataMapper> cubeMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  cubeMapper->SetInputConnection(cubeSource->GetOutputPort());
  vtkSmartPointer<vtkActor> cubeActor = 
      vtkSmartPointer<vtkActor>::New();
  cubeActor->SetMapper(cubeMapper);
  cubeActor->GetProperty()->SetOpacity(0.5);
  */
  
  //Create a mapper and actor for cube
  // sample the function
  vtkSmartPointer<vtkSampleFunction> sample = 
      vtkSmartPointer<vtkSampleFunction>::New();
  sample->SetSampleDimensions(50,50,50);
  sample->SetImplicitFunction(sphere);
  double value = 2.0;
  double xmin = -value, xmax = value, ymin = -value, ymax = value, zmin = -value, zmax = value;
  sample->SetModelBounds(xmin, xmax, ymin, ymax, zmin, zmax);
    
  //create the 0 isosurface
  vtkSmartPointer<vtkContourFilter> contours = 
      vtkSmartPointer<vtkContourFilter>::New();
  contours->SetInput(sample->GetOutput());
  contours->GenerateValues(1, 1, 1);
  
  // map the contours to graphical primitives
  vtkSmartPointer<vtkPolyDataMapper> contourMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  contourMapper->SetInput(contours->GetOutput());
  contourMapper->SetScalarRange(0.0, 1.2);
  
  // create an actor for the sphere
  vtkSmartPointer<vtkActor> sphereActor = 
      vtkSmartPointer<vtkActor>::New();
  sphereActor->SetMapper(contourMapper);
  
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
  renderer->AddActor(sphereActor);
  renderer->SetBackground(1,1,1); // Background color white
 
  //Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
