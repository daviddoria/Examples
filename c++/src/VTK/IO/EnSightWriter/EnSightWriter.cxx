#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkUnstructuredGrid.h>
#include <vtkEnSightWriter.h>
#include <vtkSphereSource.h>
#include <vtkExtractUnstructuredGrid.h>

int main ( int argc, char *argv[] )
{
  if ( argc != 2 )
    {
    cout << "Required parameters: OutputFilename.ensight" << endl;
    return EXIT_FAILURE;
    }

  std::string outputFilename = argv[1];

  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  
  vtkSmartPointer<vtkExtractUnstructuredGrid> extractUg = 
      vtkSmartPointer<vtkExtractUnstructuredGrid>::New();
  extractUg->SetInputConnection(sphereSource->GetOutputPort());
  extractUg->Update();
  
  vtkUnstructuredGrid* ug = extractUg->GetOutput();
  //vtkUnstructuredGrid* ug = vtkUnstructuredGrid::SafeDownCast(sphereSource->GetOutput());
  cout << ug->GetNumberOfPoints() << " points." << endl;
  cout << ug->GetNumberOfCells() << " cells." << endl;
  
  vtkSmartPointer<vtkEnSightWriter> writer = 
      vtkSmartPointer<vtkEnSightWriter>::New();
  writer->SetFileName(outputFilename.c_str());
  writer->SetInput(ug);
  writer->Write();

  return EXIT_SUCCESS;
}