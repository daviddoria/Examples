#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkIdList.h>
#include <vtkKdTree.h>
#include <vtkUnsignedCharArray.h>

#include <iostream>
#include <string>
#include <sstream>

int main(int argc, char *argv[])
{
	if(argc != 5)
	{
		std::cout << "argc = " << argc << std::endl;
		std::cout << "Required arguments: InputFilename OutputFilename PointID k" << std::endl;
		exit(-1);
	}
	
	std::string InputFilename = argv[1];
	std::string OutputFilename = argv[2];
	std::string strPointID = argv[3];
	std::string strK = argv[4];
	
	std::stringstream ssPointID;
	ssPointID << strPointID;
	unsigned int PointID;
	ssPointID >> PointID;
	
	std::stringstream ssK;
	ssK << strK;
	unsigned int k;
	ssK >> k;
	
	vtkXMLPolyDataReader* reader = vtkXMLPolyDataReader::New();
	reader->SetFileName(InputFilename.c_str());
	reader->Update();
	
	vtkPolyData* input = reader->GetOutput();
	vtkPoints* Points = input->GetPoints();
	
	//Create the tree
	vtkKdTree* PointTree = vtkKdTree::New();
	PointTree->BuildLocatorFromPoints(Points);
	
	// Find the k closest points
	double TestPoint[3];
	Points->GetPoint(PointID, TestPoint);
	
	vtkIdList* Result = vtkIdList::New();
	PointTree->FindClosestNPoints(k, TestPoint, Result);
	
	vtkUnsignedCharArray* Colors = vtkUnsignedCharArray::New();
	Colors->SetNumberOfComponents(3);
	Colors->SetName("Colors");
	
	unsigned char Red[3] = {255, 0, 0};
	unsigned char Green[3] = {0, 255, 0};
	
	for(unsigned int i = 0; i < Points->GetNumberOfPoints(); i++)
	{
		Colors->InsertNextTupleValue(Red);
	}
	
	for(int i = 0; i < k; i++)
	{
		int point_ind = static_cast<unsigned int>(Result->GetId(i));
		Colors->SetTupleValue(point_ind, Green);
		
		double p[3];
		Points->GetPoint(point_ind, p);
		std::cout << "Closest point " << i << ": Point " << point_ind << ": (" << p[0] << ", " << p[1] << ", " << p[2] << ")" << std::endl;
	}
	
	input->GetPointData()->SetVectors(Colors);
	
	vtkXMLPolyDataWriter* writer = vtkXMLPolyDataWriter::New();
	writer->SetFileName(OutputFilename.c_str());
	writer->SetInput(input);
	writer->Write();	
	
	return 0;
}

