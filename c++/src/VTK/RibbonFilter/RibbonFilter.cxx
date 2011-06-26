#include <vtkSmartPointer.h>
#include <vtkLine.h>
#include <vtkCellArray.h>
#include <vtkRibbonFilter.h>
#include <vtkLineSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

int main(int argc, char *argv[])
{
  //create two points, P0 and P1
  double P0[3] = {1.0, 0.0, 0.0};
  double P1[3] = {0.0, 1.0, 0.0};

  //add the two poitns to a vtkPoints object
  vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
  pts->InsertNextPoint(P0);
  pts->InsertNextPoint(P1);

  //create a line between the two points
  vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
  line->GetPointIds()->SetId(0,0);
  line->GetPointIds()->SetId(1,1);

  //create a cell array to store the line in
  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
  lines->InsertNextCell(line);

  //create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> pdata = vtkSmartPointer<vtkPolyData>::New();

  //add the points to the dataset
  pdata->SetPoints(pts);

  //add the lines to the dataset
  pdata->SetLines(lines);

  //Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> lineMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  lineMapper->SetInput(pdata);
  vtkSmartPointer<vtkActor> lineActor = 
      vtkSmartPointer<vtkActor>::New();
  lineActor->SetMapper(lineMapper);
  
  //create a ribbon around the line
  vtkSmartPointer<vtkRibbonFilter> ribbonFilter = 
      vtkSmartPointer<vtkRibbonFilter>::New();
  ribbonFilter->SetInput(pdata);
  ribbonFilter->SetWidth(2.0);
  ribbonFilter->SetAngle(20.0);
  ribbonFilter->Update();
  
  //Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> ribbonMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  ribbonMapper->SetInputConnection(ribbonFilter->GetOutputPort());
  vtkSmartPointer<vtkActor> ribbonActor = 
      vtkSmartPointer<vtkActor>::New();
  ribbonActor->SetMapper(ribbonMapper);
 
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
  renderer->AddActor(ribbonActor);
  renderer->AddActor(lineActor);
 
  //Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}

