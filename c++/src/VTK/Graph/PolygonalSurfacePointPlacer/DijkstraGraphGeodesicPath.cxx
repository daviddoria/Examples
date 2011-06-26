#include "vtkSmartPointer.h"
#include "vtkPolyData.h"
#include "vtkSphereSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkProperty.h"
#include "vtkCamera.h"
#include "vtkPolyDataCollection.h"
#include "vtkTriangleFilter.h"
#include "vtkContourWidget.h"
#include "vtkOrientedGlyphContourRepresentation.h"
#include "vtkPolygonalSurfacePointPlacer.h"
#include "vtkPolygonalSurfaceContourLineInterpolator.h"

int main(int argc, char*argv[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  
  // The Dijkistra interpolator will not accept cells that aren't triangles
  vtkSmartPointer<vtkTriangleFilter> triangleFilter = 
      vtkSmartPointer<vtkTriangleFilter>::New();
  triangleFilter->SetInputConnection( sphereSource->GetOutputPort() );
  triangleFilter->Update();
  
  vtkSmartPointer<vtkPolyData> pd = triangleFilter->GetOutput();
  
   //Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(triangleFilter->GetOutputPort());
 
  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  
  /*
  vtkSmartPointer<vtkPolyDataNormals> normals = 
      vtkSmartPointer<vtkPolyDataNormals>::New();
  normals->SetInputConnection(triangleFilter->GetOutputPort());
  normals->Update();
  */
  // Create the RenderWindow, Renderer and the DEM + path actors.
 
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);
  
  // Add the actors to the renderer, set the background and size
  
  renderer->AddActor(actor);

  // Here comes the contour widget stuff.....

  vtkSmartPointer<vtkContourWidget> contourWidget = 
      vtkSmartPointer<vtkContourWidget>::New();
  contourWidget->SetInteractor(interactor);
  vtkSmartPointer<vtkOrientedGlyphContourRepresentation> rep = 
      vtkOrientedGlyphContourRepresentation::SafeDownCast(
                        contourWidget->GetRepresentation());
  rep->GetLinesProperty()->SetColor(1, 0.2, 0);
  rep->GetLinesProperty()->SetLineWidth(3.0);

  vtkSmartPointer<vtkPolygonalSurfacePointPlacer> pointPlacer =
        vtkSmartPointer<vtkPolygonalSurfacePointPlacer>::New();
  pointPlacer->AddProp(actor);
  pointPlacer->GetPolys()->AddItem( pd );
  rep->SetPointPlacer(pointPlacer);

  vtkSmartPointer<vtkPolygonalSurfaceContourLineInterpolator> interpolator =
    vtkSmartPointer<vtkPolygonalSurfaceContourLineInterpolator>::New();
  interpolator->GetPolys()->AddItem( pd );
  rep->SetLineInterpolator(interpolator);
  
  renderWindow->Render();
  interactor->Initialize();

  contourWidget->EnabledOn();

  interactor->Start();
    
  return EXIT_SUCCESS;
}
