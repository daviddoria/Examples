#include <vtkSmartPointer.h>
#include <vtkIdList.h>
#include <vtkProperty.h>
#include <vtkStructuredGrid.h>
#include <vtkXMLStructuredGridWriter.h>
#include <vtkMath.h>
#include <vtkDataSetMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkStructuredGridGeometryFilter.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLStructuredGridWriter.h>

int main(int, char *[])
{
  // Create a grid
  vtkSmartPointer<vtkStructuredGrid> structuredGrid =
    vtkSmartPointer<vtkStructuredGrid>::New();

  vtkSmartPointer<vtkPoints> points =
    vtkSmartPointer<vtkPoints>::New();
  
  unsigned int gridSize = 8;
  unsigned int counter = 0;
  // Create a 5x5 grid of points
  for(unsigned int j = 0; j < gridSize; j++)
    {
    for(unsigned int i = 0; i < gridSize; i++)
      {
      if(i == 3 && j == 3) // Make one point higher than the rest
        {
        points->InsertNextPoint(i, j, 2);
	std::cout << "The different point is number " << counter << std::endl;
        }
      else
        {
        points->InsertNextPoint(i, j, 0); // Make most of the points the same height
        }
      counter++;
      }
    }

  // Specify the dimensions of the grid
  structuredGrid->SetDimensions(gridSize,gridSize,1);

  structuredGrid->SetPoints(points);

  vtkSmartPointer<vtkStructuredGridGeometryFilter> geometryFilter =
    vtkSmartPointer<vtkStructuredGridGeometryFilter>::New();
  geometryFilter->SetInputConnection(structuredGrid->GetProducerPort());
  geometryFilter->Update();

  vtkSmartPointer<vtkXMLPolyDataWriter> gridWriter =
    vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  gridWriter->SetInputConnection(geometryFilter->GetOutputPort());
  gridWriter->SetFileName("FullGrid.vtp");
  gridWriter->Write();

  structuredGrid->BlankPoint(27);
  structuredGrid->Modified();

  vtkSmartPointer<vtkXMLPolyDataWriter> writer =
    vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetInputConnection(geometryFilter->GetOutputPort());
  writer->SetFileName("BlankedGrid.vtp");
  writer->Write();

  // Determine which cells were blanked by inspecting the polydata
  for(vtkIdType i = 0; i < geometryFilter->GetOutput()->GetNumberOfPoints(); i++)
    {
    vtkSmartPointer<vtkIdList> cellIds =
      vtkSmartPointer<vtkIdList>::New();
    geometryFilter->GetOutput()->GetPointCells(i, cellIds);
    std::cout << "Point " << i << " is attached to " << cellIds->GetNumberOfIds() << " cells." << std::endl;
    }

  vtkSmartPointer<vtkDataSetMapper> pointsMapper =
    vtkSmartPointer<vtkDataSetMapper>::New();
  pointsMapper->SetInputConnection(geometryFilter->GetOutputPort());

  vtkSmartPointer<vtkActor> pointsActor =
    vtkSmartPointer<vtkActor>::New();
  pointsActor->SetMapper(pointsMapper);
  pointsActor->GetProperty()->SetColor(1,0,0);
  pointsActor->GetProperty()->SetPointSize(3);
  pointsActor->GetProperty()->SetRepresentationToPoints();

  // Create a mapper and actor
  vtkSmartPointer<vtkDataSetMapper> gridMapper =
    vtkSmartPointer<vtkDataSetMapper>::New();
  gridMapper->SetInputConnection(structuredGrid->GetProducerPort());

  vtkSmartPointer<vtkActor> gridActor =
    vtkSmartPointer<vtkActor>::New();
  gridActor->SetMapper(gridMapper);
  gridActor->GetProperty()->EdgeVisibilityOn();
  gridActor->GetProperty()->SetEdgeColor(0,0,1);

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
  renderer->AddActor(gridActor);
  //renderer->AddActor(pointsActor);
  renderer->SetBackground(.3, .6, .3); // Background color green

  // Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
