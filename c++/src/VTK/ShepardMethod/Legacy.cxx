#include <vtkShepardMethod.h>
#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkSphereSource.h>
#include <vtkXMLImageDataWriter.h>

int main (int argc, char *argv[])
{
  if(argc != 3)
    {
    cout << "Required arguments: InputFilename OutputFilename" << endl;
    return EXIT_FAILURE;
    }
  
  vtkstd::string inputFilename = argv[1];
  vtkstd::string outputFilename = argv[2];
	
  /*
  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(inputFilename.c_str());
  reader->Update();
  vtkPolyData* input = reader->GetOutput()
  */
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  
  vtkPolyData* input = sphereSource->GetOutput();
  
  vtkSmartPointer<vtkShepardMethod> shepard = vtkSmartPointer<vtkShepardMethod>::New();
  shepard->SetInput(input);
  //shepard->SetModelBounds(0, 1, 0, 1, .1, .5);
  shepard->SetSampleDimensions(20, 20, 20);
//Error - scalars must be defined??
  shepard->Update();
  
  vtkSmartPointer<vtkXMLImageDataWriter> writer = vtkSmartPointer<vtkXMLImageDataWriter>::New();
  writer->SetFileName(outputFilename.c_str());
  writer->SetInput(shepard->GetOutput());
  writer->Write();
  
  return EXIT_SUCCESS;
}