#include <vtkTransformPolyDataFilter.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkTriangle.h>
#include <iostream>

int main(int argc, char *argv[])
{
	

	vtkSmartPointer<vtkPoints> SourcePoints = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> SourceVertices = vtkSmartPointer<vtkCellArray>::New();
	
	//add three points and vertices
	double SourcePoint1[3] = {1.0, 0.0, 0.0};
	vtkIdType pid[1]; //to store the point ID that results from InsertNextPoint
	pid[0] = SourcePoints->InsertNextPoint(SourcePoint1);
	SourceVertices->InsertNextCell(1,pid);
	
	double SourcePoint2[3] = {0.0, 1.0, 0.0};
	pid[0] = SourcePoints->InsertNextPoint(SourcePoint2);
	SourceVertices->InsertNextCell(1,pid);
	
	double SourcePoint3[3] = {0.0, 0.0, 1.0};
	pid[0] = SourcePoints->InsertNextPoint(SourcePoint3);
	SourceVertices->InsertNextCell(1,pid);
		
	//create a polydata to add everything to
	vtkSmartPointer<vtkPolyData> Source = vtkPolyData::New();
	Source->SetPoints(SourcePoints);
	Source->SetVerts(SourceVertices);

	//create a triangle
	vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
	triangle->GetPointIds()->SetId(0,0);
	triangle->GetPointIds()->SetId(1,1);
	triangle->GetPointIds()->SetId(2,2);
	
	//add the triangle to a cell array
	vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();
	triangles->InsertNextCell(triangle);

	//add the triangle to the cell array
	Source->SetPolys(triangles);

	//write original polydata
	{
	vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
	writer->SetFileName("orig.vtp");
	writer->SetInput(Source);
	writer->Write();
	}
	
	//output original points
	for(unsigned int i = 0; i < 3; i++)
	{
		double origpoint[3];
		SourcePoints->GetPoint(i, origpoint);
		std::cout << "Original point: (" << origpoint[0] << ", " << origpoint[1] << ", " << origpoint[2] << ")" << std::endl;
	}
	
	vtkSmartPointer<vtkTransform> Translation = vtkSmartPointer<vtkTransform>::New();
	Translation->Translate(1.0, 2.0, 3.0);
	
	vtkSmartPointer<vtkTransformPolyDataFilter> TranslateFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	TranslateFilter->SetInput(Source);
	TranslateFilter->SetTransform(Translation);
	TranslateFilter->Update();
	
	vtkSmartPointer<vtkPolyData> Translated = TranslateFilter->GetOutput();

	//write transformed polydata
	{
		vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
		writer->SetFileName("Transformed.vtp");
		writer->SetInput(Translated);
		writer->Write();
	}
	
	//output transformed points
	for(unsigned int i = 0; i < 3; i++)
	{
		double point[3];
		Translated->GetPoint(i, point);
		std::cout << "Translated point: (" << point[0] << ", " << point[1] << ", " << point[2] << ")" << std::endl;
	}
	
	return 0;
}
