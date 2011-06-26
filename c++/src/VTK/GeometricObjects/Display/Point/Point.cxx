#include <vtkSmartPointer.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkProperty.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

int main(int argc, char *argv[])
{
  //Create the geometry of a point (the coordinate)
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  const float p[3] = {1.0, 2.0, 3.0};
  points->InsertNextPoint(p);
  
  vtkSmartPointer<vtkPolyData> pointsPolyData = 
      vtkSmartPointer<vtkPolyData>::New();
  pointsPolyData->SetPoints(points);
  
  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = 
      vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputConnection(pointsPolyData->GetProducerPort());
  
  //Create an actor and mapper
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  //mapper->SetInput(point);
  //mapper->SetInputConnection(pointsPolyData->GetProducerPort());
  mapper->SetInputConnection(glyphFilter->GetOutputPort());

  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetPointSize(5);
  actor->GetProperty()->SetColor(0.0, 1.0, 0.0); //(R,G,B)
  
  //Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  //Add the actors to the scene
  renderer->AddActor(actor);
  
  //Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
