#include "itkImage.h"
#include "itkImageFileWriter.h"

#include "vtkSmartPointer.h"
#include "vtkXMLPolyDataReader.h"
#include "vtkPolyData.h"

#include <iostream>
#include <vector>
#include <string>
#include <sstream>

struct Point
{
double x,y,z;	
};

Point MinCorner(std::vector<Point> &Points);
Point MaxCorner(std::vector<Point> &Points);

int main(int argc, char *argv[])
{
	std::string InputFile = argv[1];
	std::string OutputFile = argv[2];
	std::string strNumBins = argv[3];
	
	std::stringstream ssNumBins;
	ssNumBins << strNumBins;
	
	unsigned int NumBins;
	ssNumBins >> NumBins;
	
	//read the points
	vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
	reader->SetFileName(InputFile.c_str());
	reader->Update();
	vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
	
	//get points
	vtkIdType idNumPointsInFile = polydata->GetNumberOfPoints();
	unsigned int NumPointsInFile = static_cast<unsigned int> (idNumPointsInFile);
	
	std::vector<Point> Points;
	
	double point[3];
	for(unsigned int i = 0; i < NumPointsInFile; i++)
	{
		polydata->GetPoint(i, point);
		Point P;
		P.x = point[0];
		P.y = point[1];
		P.z = point[2];
		
		Points.push_back(P);
	}
	
	Point MinPoint = MinCorner(Points);
	Point MaxPoint = MaxCorner(Points);
	
	typedef itk::Image< float, 3 > ImageType;
	ImageType::Pointer image = ImageType::New();
	
	ImageType::IndexType start;
	start[0] = 0;
	start[1] = 0;
	start[2] = 0;
	
	
	ImageType::SizeType  size;
	size[0] = NumBins;
	size[1] = NumBins;
	size[2] = NumBins;
	
	ImageType::RegionType region;
  	region.SetSize( size );
	region.SetIndex( start );
	image->SetRegions( region );
	
	//does not break it
	ImageType::PointType origin;
	origin[0] = MinPoint.x;
	origin[1] = MinPoint.y;
	origin[2] = MinPoint.z;
	image->SetOrigin(origin);
	
	image->Allocate();
		
	//fixed number of voxels in each dimension
	double xdiff = MaxPoint.x - MinPoint.x;
	double ydiff = MaxPoint.y - MinPoint.y;
	double zdiff = MaxPoint.z - MinPoint.z;
	
	ImageType::SpacingType spacing;
	spacing[0] = xdiff/static_cast<double> (NumBins);
	spacing[1] = ydiff/static_cast<double> (NumBins);
	spacing[2] = zdiff/static_cast<double> (NumBins);
	image->SetSpacing(spacing);
		
	
	for(unsigned int i = 0; i < Points.size(); i++)
	{
		//set the nearest voxel to a point to 4.0
		ImageType::PointType point;
		point[0] = Points[i].x;
		point[1] = Points[i].y;
		point[2] = Points[i].z;
		
		ImageType::IndexType ClosestPixel;
		
		bool isInside = image->TransformPhysicalPointToIndex(point, ClosestPixel);
		if(isInside)
		{
			image->SetPixel( ClosestPixel, 1.0 );
			//std::cout << "The closest pixel was at index: " << ClosestPixel << std::endl;
		}
		else
		{
			//std::cout << "Requested point is not inside the image!" << std::endl;
		}
	}
	
	
	typedef itk::ImageFileWriter< ImageType > WriterType;
	WriterType::Pointer writer = WriterType::New();

	writer->SetFileName(OutputFile.c_str());

	writer->SetInput(  image  );
	writer->Update();
	
	return 0;
}

Point MinCorner(std::vector<Point> &Points)
{
	
	double x = 1e5;
	double y = 1e5;
	double z = 1e5;
	
	for(unsigned int i = 0; i < Points.size(); i++)
	{
		if(Points[i].x < x)
			x = Points[i].x;
		if(Points[i].y < y)
			y = Points[i].y;
		if(Points[i].z < z)
			z = Points[i].z;
	}
		
	Point Corner;
	Corner.x = x;
	Corner.y = y;
	Corner.z = z;
	
	return Corner;
}


Point MaxCorner(std::vector<Point> &Points)
{
	
	double x = -1e5;
	double y = -1e5;
	double z = -1e5;
	
	for(unsigned int i = 0; i < Points.size(); i++)
	{
		if(Points[i].x > x)
			x = Points[i].x;
		if(Points[i].y > y)
			y = Points[i].y;
		if(Points[i].z > z)
			z = Points[i].z;
	}
	
	Point Corner;
	Corner.x = x;
	Corner.y = y;
	Corner.z = z;
	
	return Corner;
}