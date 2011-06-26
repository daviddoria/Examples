#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkCleanPolyData.h>
#include <vtkAppendPolyData.h>

int main(int argc, char *argv[])
{
  if(argc != 4)
    {
    std::cout << "argc = " << argc << std::endl;
    std::cout << "Required arguments: File1 File2 CombiledFile" << std::endl;
    exit(-1);
    }
  
  std::string InputFilename1 = argv[1];
  std::string InputFilename2 = argv[2];
  std::string OutputFilename = argv[3];
  
  //You can append the two meshes with vtkAppendPolyData followed by a
  //vtkCleanPolyData to remove duplicate points.
          
  vtkSmartPointer<vtkXMLPolyDataReader> reader1 = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader1->SetFileName(InputFilename1.c_str());
  reader1->Update();
  vtkPolyData* polydata1 = reader1->GetOutput();
  
  vtkSmartPointer<vtkXMLPolyDataReader> reader2 = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader2->SetFileName(InputFilename2.c_str());
  reader2->Update();
  vtkPolyData* polydata2 = reader2->GetOutput();
  
  vtkSmartPointer<vtkAppendPolyData> appendFilter = vtkSmartPointer<vtkAppendPolyData>::New();
  appendFilter->AddInput(polydata1);
  appendFilter->AddInput(polydata2);
  
  vtkPolyData* polydataCombined = appendFilter->GetOutput();
  
  vtkSmartPointer<vtkCleanPolyData> cleanFilter = vtkSmartPointer<vtkCleanPolyData>::New();
  cleanFilter->SetInput(polydataCombined);
  vtkPolyData* polydataCleaned = cleanFilter->GetOutput();
  
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetInput(polydataCleaned);
  writer->SetFileName(OutputFilename.c_str());
  writer->Update();
      
  
  return 0;
}
