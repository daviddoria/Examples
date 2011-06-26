#include <vtkSmartPointer.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkQuad.h>
#include <vtkPolyData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>

int main(int argc, char *argv[])
{
  //Create four points (must be in counter clockwise order)
  double p0[3] = {0.0, 0.0, 0.0};
  double p1[3] = {1.0, 0.0, 0.0};
  double p2[3] = {1.0, 1.0, 0.0};
  double p3[3] = {0.0, 1.0, 1.0};
      
  //Add the points to a vtkPoints object
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(p0);
  points->InsertNextPoint(p1);
  points->InsertNextPoint(p2);
  points->InsertNextPoint(p3);
  
  vtkSmartPointer<vtkPolyData> pointsPolydata = 
      vtkSmartPointer<vtkPolyData>::New();
  pointsPolydata->SetPoints(points);
  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = 
      vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInput(pointsPolydata);
  glyphFilter->Update();
  
  //setup actor and mapper for points
  vtkSmartPointer<vtkPolyDataMapper> pointsMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  pointsMapper->SetInputConnection(glyphFilter->GetOutputPort());
  
  vtkSmartPointer<vtkActor> pointsActor = 
      vtkSmartPointer<vtkActor>::New();
  pointsActor->SetMapper(pointsMapper);
  pointsActor->GetProperty()->SetPointSize(10);
  
  //create a line between the two points
  vtkSmartPointer<vtkQuad> quad = 
      vtkSmartPointer<vtkQuad>::New();
  quad->GetPointIds()->SetId(0,0);
  quad->GetPointIds()->SetId(1,1);
  quad->GetPointIds()->SetId(2,2);
  quad->GetPointIds()->SetId(3,3);
  
  //create a cell array to store the line in
  vtkSmartPointer<vtkCellArray> quads = 
      vtkSmartPointer<vtkCellArray>::New();
  quads->InsertNextCell(quad);

  //create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> polydata = 
      vtkSmartPointer<vtkPolyData>::New();

  //add the points and lines to the dataset
  polydata->SetPoints(points);
  polydata->SetPolys(quads);

  //setup actor and mapper
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInput(polydata);
  
  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  
  //setup render window, renderer, and interactor
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
  
  renderer->AddActor(actor);
  renderer->AddActor(pointsActor);
  
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}

