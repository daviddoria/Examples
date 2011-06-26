#include <vtkSmartPointer.h>
#include <vtkSurfaceReconstructionFilter.h>
#include <vtkProgrammableSource.h>
#include <vtkContourFilter.h>
#include <vtkReverseSense.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkPolyData.h>
#include <vtkCamera.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkMath.h>
#include <cmath>


vtkSmartPointer<vtkPoints> readPoints();

int main(int argc, char **argv)
{

	// Read some points
  vtkSmartPointer<vtkPoints> points = readPoints();
  vtkSmartPointer<vtkPolyData> polydata = 
      vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);

  // Construct the surface and create isosurface.	
  vtkSmartPointer<vtkSurfaceReconstructionFilter> surf = 
      vtkSmartPointer<vtkSurfaceReconstructionFilter>::New();
  surf->SetInput(polydata);

  vtkSmartPointer<vtkContourFilter> cf = 
      vtkSmartPointer<vtkContourFilter>::New();
  cf->SetInputConnection(surf->GetOutputPort());
  cf->SetValue(0, 0.0);

  // Sometimes the contouring algorithm can create a volume whose gradient
  // vector and ordering of polygon (using the right hand rule) are
  // inconsistent. vtkReverseSense cures this problem.
  vtkSmartPointer<vtkReverseSense> reverse = 
      vtkSmartPointer<vtkReverseSense>::New();
  reverse->SetInputConnection(cf->GetOutputPort());
  reverse->ReverseCellsOn();
  reverse->ReverseNormalsOn();

  vtkSmartPointer<vtkPolyDataMapper> map = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  map->SetInputConnection(reverse->GetOutputPort());
  map->ScalarVisibilityOff();

  vtkSmartPointer<vtkActor> surfaceActor = 
      vtkSmartPointer<vtkActor>::New();
  surfaceActor->SetMapper(map);
  surfaceActor->GetProperty()->SetDiffuseColor(1.0000, 0.3882, 0.2784);
  surfaceActor->GetProperty()->SetSpecularColor(1, 1, 1);
  surfaceActor->GetProperty()->SetSpecular(.4);
  surfaceActor->GetProperty()->SetSpecularPower(50);

  // Create the RenderWindow, Renderer and both Actors
  vtkSmartPointer<vtkRenderer> ren = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renWin = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renWin->AddRenderer(ren);
  vtkSmartPointer<vtkRenderWindowInteractor> iren = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  iren->SetRenderWindow(renWin);

  // Add the actors to the renderer, set the background and size
  ren->AddActor(surfaceActor);
  ren->SetBackground(1, 1, 1);
  renWin->SetSize(400, 400);
  ren->GetActiveCamera()->SetFocalPoint(0, 0, 0);
  ren->GetActiveCamera()->SetPosition(1, 0, 0);
  ren->GetActiveCamera()->SetViewUp(0, 0, 1);
  ren->ResetCamera();
  ren->GetActiveCamera()->Azimuth(20);
  ren->GetActiveCamera()->Elevation(30);
  ren->GetActiveCamera()->Dolly(1.2);
  ren->ResetCameraClippingRange();

  iren->Initialize();
  renWin->Render();
  iren->Start();

  return EXIT_SUCCESS;

}



vtkSmartPointer<vtkPoints> readPoints()
{
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  
  float x, y, z;
  // generate random points on unit sphere
  for(int i=0; i<100; i++) 
    {
      double phi, theta,u,v;
      u = vtkMath::Random(0.0,1.0);
      v = vtkMath::Random(0.0,1.0);
      phi = 2.0*vtkMath::Pi()*u;
      theta = acos(2.0*v-1.0);

      x = std::cos(phi)*std::sin(theta);
      y = std::sin(phi)*std::sin(theta);
      z = std::cos(theta);

      points->InsertNextPoint(x, y, z);
    }
  return points;
}
