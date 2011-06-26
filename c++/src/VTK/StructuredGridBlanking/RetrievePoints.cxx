#include <vtkSmartPointer.h>
#include <vtkStructuredGrid.h>
#include <vtkXMLStructuredGridWriter.h>
#include <vtkMath.h>
#include <vtkDataSetMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

int main(int, char *[])
{
  // Create a grid
  vtkSmartPointer<vtkStructuredGrid> structuredGrid =
    vtkSmartPointer<vtkStructuredGrid>::New();

  vtkSmartPointer<vtkPoints> points =
    vtkSmartPointer<vtkPoints>::New();
  for(unsigned int j = 0; j < 4; j++)
    {
    for(unsigned int i = 0; i < 4; i++)
      {
      if(i == 2 && j == 2)
        {
        points->InsertNextPoint(i, j, 2);
        }
      else
        {
        points->InsertNextPoint(i, j, 0);
        }
      }
    }

  // Specify the dimensions of the grid
  structuredGrid->SetDimensions(4,4,1);
  structuredGrid->BlankPoint(3);
  structuredGrid->SetPoints(points);

  int* dims = structuredGrid->GetDimensions();

  // Retrieve the entries from the grid and print them to the screen
  unsigned int counter = 0;

  for (int k = 0; k < dims[2]; k++)
    {
    for (int j = 0; j < dims[1]; j++)
      {
      for (int i = 0; i < dims[0]; i++)
        {
        double pLinear[3];
        structuredGrid->GetPoint(counter, pLinear);

        double pGrid[3];
        structuredGrid->GetPoint(i, j, k, pGrid);

        std::cout << "pLinear : " << pLinear[0] << " " << pLinear[1] << " " << pLinear[2] << std::endl;
        std::cout << "pGrid: " << pGrid[0] << " " << pGrid[1] << " " << pGrid[2] << std::endl;

        counter++;

        std::cout << endl << std::endl;
        }

      }
    }

  // Create a mapper and actor
  vtkSmartPointer<vtkDataSetMapper> mapper =
    vtkSmartPointer<vtkDataSetMapper>::New();
  mapper->SetInputConnection(structuredGrid->GetProducerPort());

  vtkSmartPointer<vtkActor> actor =
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  // Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actor to the scene
  renderer->AddActor(actor);
  renderer->SetBackground(.3, .6, .3); // Background color green

  // Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}