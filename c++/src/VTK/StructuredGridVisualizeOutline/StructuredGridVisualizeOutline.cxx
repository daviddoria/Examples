#include <vtkSmartPointer.h>
#include <vtkStructuredGrid.h>
#include <vtkXMLStructuredGridWriter.h>
#include <vtkMath.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkStructuredGridOutlineFilter.h>

int main(int argc, char *argv[])
{
  //create a grid
  vtkSmartPointer<vtkStructuredGrid> structuredGrid = 
      vtkSmartPointer<vtkStructuredGrid>::New();
  
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  unsigned int numi = 2;
  unsigned int numj = 3;
  unsigned int numk = 2;
  
  for(unsigned int k = 0; k < numk; k++)
    {
    for(unsigned int j = 0; j < numj; j++)
      {
      for(unsigned int i = 0; i < numi; i++)
        {
        points->InsertNextPoint(i, j, k);
        }
      }
    }
  
  //specify the dimensions of the grid
  structuredGrid->SetDimensions(numi, numj, numk);
  structuredGrid->SetPoints(points);
  
  cout << "There are " << structuredGrid->GetNumberOfPoints() << " points." << endl;
  cout << "There are " << structuredGrid->GetNumberOfCells() << " cells." << endl;

  vtkSmartPointer<vtkStructuredGridOutlineFilter> outlineFilter = 
      vtkSmartPointer<vtkStructuredGridOutlineFilter>::New();
  outlineFilter->SetInputConnection(structuredGrid->GetProducerPort());
  outlineFilter->Update();
  
  //Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(outlineFilter->GetOutputPort());
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
  //renderer->SetBackground(1,1,1); // Background color white
 
  //Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}