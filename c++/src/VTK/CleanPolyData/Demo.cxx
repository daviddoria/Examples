#include <vtkPolyData.h>
#include <vtkCleanPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>

#include <sstream>

int main(int argc, char *argv[])
{
  if(argc != 4)
    {
    cout << "Required arguments: InputFilename OutputFilename spacing" << endl;
    return EXIT_FAILURE;
    }
  
  vtkstd::string InputFilename = argv[1];
  vtkstd::string OutputFilename = argv[2];
  vtkstd::string strSpacing = argv[3];
  
  std::stringstream ss;
  ss << strSpacing;
  double spacing;
  ss >> spacing;
  
  vtkSmartPointer<vtkXMLPolyDataReader> reader = 
      vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(InputFilename.c_str());
  reader->Update();
  
  vtkSmartPointer<vtkCleanPolyData> cleanPolyData = 
      vtkSmartPointer<vtkCleanPolyData>::New();
  cleanPolyData->SetInput(Reader->GetOutput());
  cleanPolyData->SetTolerance(spacing);
  cleanPolyData->Update();
  
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(OutputFilename.c_str());
  writer->SetInput(CleanPolyData->GetOutput());
  writer->Write();
  
  return 0;
}
