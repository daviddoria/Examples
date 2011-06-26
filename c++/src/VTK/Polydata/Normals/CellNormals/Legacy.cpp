#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkTriangle.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>

#include <iostream>
#include <string>

int main(int argc, char *argv[])
{

	//setup points
	vtkPoints* points = vtkPoints::New();
	points->InsertNextPoint(1.0, 0.0, 0.0);
	points->InsertNextPoint(0.0, 0.0, 0.0);
	points->InsertNextPoint(0.0, 1.0, 0.0);
	
	//setup triangle
	vtkCellArray* triangles = vtkCellArray::New();
	vtkTriangle* triangle = vtkTriangle::New();
	triangle->GetPointIds()->SetId(0,0);
	triangle->GetPointIds()->SetId(1,1);
	triangle->GetPointIds()->SetId(2,2);
	triangles->InsertNextCell(triangle);
	
	vtkPolyData* polydata = vtkPolyData::New();
	polydata->SetPoints(points);
	polydata->SetPolys(triangles);
	
	//count points
	vtkIdType NumPoints = polydata->GetNumberOfPoints();
	std::cout << "There are " << NumPoints << " points." << std::endl;
	
	//count triangles
	vtkIdType NumTris = polydata->GetNumberOfPolys();
	std::cout << "There are " << NumTris << " triangles." << std::endl;
	
	//set point normals
	vtkDoubleArray* PointNormalsArray = vtkDoubleArray::New();
	PointNormalsArray->SetNumberOfComponents(3); //3d normals (ie x,y,z)
	PointNormalsArray->SetNumberOfTuples(3); //there are 3 points in the triangle
	double PN1[3] = {1.0, 0.0, 0.0};
	double PN2[3] = {0.0, 1.0, 0.0};
	double PN3[3] = {0.0, 0.0, 1.0};
	PointNormalsArray->SetTuple(0, PN1) ;
	PointNormalsArray->SetTuple(1, PN2) ;
	PointNormalsArray->SetTuple(2, PN3) ;
	
	polydata->GetPointData()->SetNormals(PointNormalsArray);

	
	//set triangle normals
	vtkDoubleArray* TriangleNormalsArray = vtkDoubleArray::New();
	TriangleNormalsArray->SetNumberOfComponents(3); //3d normals (ie x,y,z)
	TriangleNormalsArray->SetNumberOfTuples(1); //there is only 1 triangle
	double TN1[3] = {1.0, 0.0, 0.0};
	TriangleNormalsArray->SetTuple(0, TN1) ;
	
	polydata->GetCellData()->SetNormals(TriangleNormalsArray);
	
	//get point normals
	vtkSmartPointer<vtkDoubleArray> PointNormalsRetrieved = vtkDoubleArray::SafeDownCast(polydata->GetPointData()->GetNormals());
	//vtkDoubleArray* PointNormalsRetrieved = polydata->GetPointData()->GetNormals(); //cannot do this for some reason
	if(PointNormalsRetrieved)
	{ 
		std::cout << "There are " << PointNormalsRetrieved->GetNumberOfTuples() << " point normals." << std::endl;
				
		for(unsigned int i = 0; i < PointNormalsRetrieved->GetNumberOfTuples(); i++)
		{
			double PN[3];
			PointNormalsRetrieved->GetTuple(i, PN);
			std::cout << "Point normal " << i << ": " << PN[0] << " " << PN[1] << " " << PN[2] << std::endl;
		}
		
	}
	else
	{
		std::cout << "No point normals." << std::endl;
	}
	
	//get triangle normals
	vtkSmartPointer<vtkDoubleArray> TriangleNormalsRetrieved = vtkDoubleArray::SafeDownCast(polydata->GetCellData()->GetNormals());
	//vtkDoubleArray* TriangleNormalsRetrieved = polydata->GetCellData()->GetNormals(); //cannot do this for some reason
	if(TriangleNormalsRetrieved)
	{ 
		std::cout << "There are " << TriangleNormalsRetrieved->GetNumberOfTuples() << " triangle normals." << std::endl;
		for(unsigned int i = 0; i < TriangleNormalsRetrieved->GetNumberOfTuples(); i++)
		{
			double TN[3];
			TriangleNormalsRetrieved->GetTuple(i, TN);
			std::cout << "Triangle normal " << i << ": " << TN[0] << " " << TN[1] << " " << TN[2] << std::endl;
		}
	}
	else
	{
		std::cout << "No triangle normals." << std::endl;
	}
	
	
	return 0;
}
