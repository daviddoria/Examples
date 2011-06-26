#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkMultiBlockDataSet.h>
#include <vtkCompositeDataIterator.h>
#include <vtkLSDynaReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkUnstructuredGrid.h>

int main(int argc, char *argv[])
{
  // Parse command line arguments
  if(argc != 2)
    {
    std::cout << "Required arguments: Filename" << std::endl;
    return EXIT_FAILURE;
    }

  std::string filename = argv[1];
  std::cout << "Reading " << filename << std::endl;

  vtkSmartPointer<vtkLSDynaReader> reader =
    vtkSmartPointer<vtkLSDynaReader>::New();
  reader->SetDatabaseDirectory("/home/doriad/Downloads" );
  reader->UpdateInformation();

  std::cout << "Number of blocks: " << reader->GetOutput()->GetNumberOfBlocks() << std::endl;

  vtkUnstructuredGrid *ug = vtkUnstructuredGrid::SafeDownCast(reader->GetOutput()->GetBlock(0));
  std::cout << ug << std::endl;

  vtkPolyData* pd = vtkPolyData::SafeDownCast(reader->GetOutput()->GetBlock(0));
  std::cout << pd << std::endl;

  vtkCompositeDataIterator* iter = reader->GetOutput()->NewIterator();
  for (iter->InitTraversal(); !iter->IsDoneWithTraversal(); iter->GoToNextItem())
    {
    vtkDataObject* dObj = iter->GetCurrentDataObject();
    std::cout << dObj->GetClassName() << std::endl;
    }
/*
  // Visualize
  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(reader->GetOutputPort());

  vtkSmartPointer<vtkActor> actor =
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderer->AddActor(actor);
  renderer->SetBackground(.3, .6, .3); // Background color green

  renderWindow->Render();
  renderWindowInteractor->Start();
  */

  return EXIT_SUCCESS;
}
