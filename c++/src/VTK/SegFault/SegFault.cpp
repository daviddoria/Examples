#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPolyDataNormals.h>

#include <iostream>
#include <string>
#include <vector>

//std::vector<Vector> GetNormalsFromPolydata(vtkSmartPointer<vtkPolyData> polydata);
//vtkPolyData* VtpRead(const std::string &filename);
void VtpRead(const std::string &filename);
void Construct(vtkSmartPointer<vtkPolyData> pd);

int main(int argc, char *argv[])
{
	std::string filename = argv[1]; //first command line argument
	for(unsigned int i = 0; i < 10; i++)
	{
		std::cout << i << std::endl;
		//vtkPolyData* polydata = VtpRead(filename);
		VtpRead(filename);
		
		//std::cout << polydata << std::endl;
	}
	
	return 0;
}

void VtpRead(const std::string &filename)
{
	vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
	cout << "Reading " << filename << endl;
	reader->SetFileName(filename.c_str());
	reader->Update();
	
	vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
	Construct(polydata);
}

void Construct(vtkSmartPointer<vtkPolyData> polydata)
{
	vtkIdType idNumPointsInFile = polydata->GetNumberOfPoints();
	std::cout << "There are " << idNumPointsInFile << " points." << std::endl;
}

#if 0
//bool ConstructFromPolydata(vtkPolyData* polydata)
bool ConstructFromPolydata(vtkSmartPointer<vtkPolyData> polydata)
{
	//get points
	vtkIdType idNumPointsInFile = polydata->GetNumberOfPoints();
	unsigned int NumPointsInFile = static_cast<unsigned int> (idNumPointsInFile);
	
	if(!(NumPointsInFile > 0) )
	{
		return false;
	}
	
	std::vector<Point> Points_(NumPointsInFile);
	
	double point[3];
	for(unsigned int i = 0; i < NumPointsInFile; i++)
	{
		polydata->GetPoint(i, point);
		Point P;
		P.x = point[0]; P.y = point[1]; P.z = point[2];
		Points_.push_back(P);
	}
	
	//get triangles
	std::vector<std::vector<unsigned int> > VertexLists_;
	vtkIdType NumPolys = polydata->GetNumberOfPolys();
	if(NumPolys > 0)
	{
		vtkSmartPointer<vtkCellArray> TriangleCells = polydata->GetPolys();
		vtkIdType npts;
		vtkIdType *pts;

		while(TriangleCells->GetNextCell(npts, pts))
		{
			std::vector<unsigned int> List(3);
			List[0] = pts[0];
			List[1] = pts[1];
			List[2] = pts[2];
			
			VertexLists_.push_back(List);
		}	
		
	}
	
	cout << "Has Scanner location: " << HasScannerLocation(polydata) << endl;
	
	//get normals
	std::vector<Vector> Normals = GetNormalsFromPolydata(polydata);
	if(Normals.size() == NumPointsInFile)
	{
		cout << "Normals found on first pass. There are " << Normals.size() << " of them." << endl;
	}
	else
	{
		//if none of the normal types we looked for exist, then create them from the cells
		cout << "Normals NOT found on first pass, extracting..." << endl;
		cerr << "Number of points before normal extraction: " << polydata->GetNumberOfPoints() << endl;
		vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
		normalGenerator->SetInput(polydata);
		normalGenerator->Update();
		polydata = normalGenerator->GetOutput();
		Normals = GetNormalsFromPolydata(polydata);
		cout << "There were " << Normals.size() << " normals extracted." << endl;
		//cerr << "Number of points after normal extraction: " << polydata->GetNumberOfPoints() << endl;
		//we should be confident that polydata is still in good shape here...
	}
	
	cout << "Has Scanner location: " << HasScannerLocation(polydata) << endl;
	
}
#endif
