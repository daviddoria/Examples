#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkSmartPointer.h>

void FindAllData(const std::string &InputFilename);

int main(int argc, char *argv[])
{
  if(argc != 2)
    {
    cout << "Required argument: InputFilename" << endl;
    return EXIT_FAILURE;
    }
  std::string InputFilename = argv[1];
  FindAllData(InputFilename);
  
  return EXIT_SUCCESS;
}

void FindAllData(const std::string &InputFilename)
{
  vtkSmartPointer<vtkXMLPolyDataReader> reader = 
      vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(InputFilename.c_str());
  reader->Update();
  vtkPolyData* polydata = reader->GetOutput();

  unsigned int NumArrays = polydata->GetPointData()->GetNumberOfArrays();
  std::cout << "NumArrays: " << NumArrays << "\n";
  
  cout << "key:" << endl;
  //more values can be found in <VTK_DIR>/Common/vtkSetGet.h
  cout << VTK_UNSIGNED_CHAR << " unsigned char" << endl;
  cout << VTK_UNSIGNED_INT << " unsigned int" << endl;
  cout << VTK_FLOAT << " float" << endl;
  cout << VTK_DOUBLE << " double" << endl;
  
  std::vector<std::string> arrayNames;
  for(unsigned int i = 0; i < NumArrays; i++)
  {
      //the following two lines are equivalent
      //ArrayNames.push_back(polydata->GetPointData()->GetArray(i)->GetName());
      arrayNames.push_back(polydata->GetPointData()->GetArrayName(i));
      int dataTypeID = polydata->GetPointData()->GetArray(i)->GetDataType();
      cout << "Array " << i << ": " << arrayNames[i] << " (type: " << dataTypeID << ")" << endl;
  }
	
}
