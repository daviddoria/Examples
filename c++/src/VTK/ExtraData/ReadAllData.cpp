#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkXMLPolyDataReader.h>

#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>


void FindAllData(const std::string &InputFilename);

int main(int argc, char *argv[])
{
	if(argc != 2)
	{
		std::cout << "Required argument: InputFilename\n";
		exit(-1);
	}
	std::string InputFilename = argv[1];
	FindAllData(InputFilename);
	
	return 0;
}

void FindAllData(const std::string &InputFilename)
{
	vtkXMLPolyDataReader* reader = vtkXMLPolyDataReader::New();
	reader->SetFileName(InputFilename.c_str());
	reader->Update();
	vtkPolyData* polydata = reader->GetOutput();

	unsigned int NumArrays = polydata->GetPointData()->GetNumberOfArrays();
	std::cout << "NumArrays: " << NumArrays << "\n";
	
	std::vector<std::string> ArrayNames;
	for(unsigned int i = 0; i < NumArrays; i++)
	{
		//the following two lines are equivalent
		//ArrayNames.push_back(polydata->GetPointData()->GetArray(i)->GetName());
		ArrayNames.push_back(polydata->GetPointData()->GetArrayName(i));
		std::cout << "Array " << i << ": " << ArrayNames[i] << "\n";
	}
	
}
