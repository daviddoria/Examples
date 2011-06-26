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
  for(unsigned int i = 0; i < 2*3*1; i++)
    {
    points->InsertNextPoint(vtkMath::Random(0, 1), vtkMath::Random(0, 1), vtkMath::Random(0, 1));
    }

  // Specify the dimensions of the grid
  structuredGrid->SetDimensions(2,3,1);
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
        double p[3];
        structuredGrid->GetPoint(counter, p);

        double pNew[3];
        structuredGrid->GetPoint(i, j, k, pNew);

        std::cout << "P   : " << p[0] << " " << p[1] << " " << p[2] << std::endl;
        std::cout << "PNew: " << pNew[0] << " " << pNew[1] << " " << pNew[2] << std::endl;

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