#include <iostream>
#include <vector>

#include "vtkPoints.h"
#include "vtkLine.h"
#include "vtkXMLPolyDataWriter.h"
#include "vtkPolyData.h"
#include "vtkCellArray.h"
#include "vtkCellData.h"
#include "vtkUnsignedCharArray.h"

int main()
{		
	vtkPolyData* poly = vtkPolyData::New();
	
	vtkPoints* points = vtkPoints::New();
	points->InsertNextPoint(0.0, 0.0, 0.0);
	points->InsertNextPoint(1.0, 0.0, 0.0);
	points->InsertNextPoint(0.0, 1.0, 0.0);
	points->InsertNextPoint(0.0, 0.0, 1.0);
	
	vtkCellArray* lines = vtkCellArray::New();
	
	vtkLine* Line = vtkLine::New();
	//x axis
	Line->GetPointIds()->SetId(0,0);
	Line->GetPointIds()->SetId(1,1);
	lines->InsertNextCell(Line);
	
	//y axis
	Line->GetPointIds()->SetId(0,0);
	Line->GetPointIds()->SetId(1,2);
	lines->InsertNextCell(Line);
	
	//z axis
	Line->GetPointIds()->SetId(0,0);
	Line->GetPointIds()->SetId(1,3);
	lines->InsertNextCell(Line);
	
	//setup colors
	unsigned char Red[3] = {255, 0, 0};
	unsigned char Yellow[3] = {255, 255, 0};
	unsigned char Green[3] = {0, 255, 0};

	vtkUnsignedCharArray* Colors = vtkUnsignedCharArray::New();
	Colors->SetNumberOfComponents(3);
	Colors->SetName("Colors");
	Colors->InsertNextTupleValue(Red);
	Colors->InsertNextTupleValue(Yellow);
	Colors->InsertNextTupleValue(Green);

	//add points and lines to polydata
	poly->SetPoints(points);
	poly->SetLines(lines);
	
	//add colors to lines
	poly->GetCellData()->SetVectors(Colors);
	
	vtkXMLPolyDataWriter* writer = vtkXMLPolyDataWriter::New();
	writer->SetFileName("coordinate_axes.vtp");
	writer->SetInput(poly);
	writer->Write();
    
    writer->Delete();
    Colors->Delete();
    Line->Delete();
    lines->Delete();
    poly->Delete();
    points->Delete();
}
