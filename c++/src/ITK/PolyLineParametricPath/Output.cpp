#include "itkPolyLineParametricPath.h"

#include <cstdlib> // for drand48
#include <fstream>

#include <vtkCellArray.h>
#include <vtkLine.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataWriter.h>

int main(int argc, char *argv[])
{
  typedef itk::PolyLineParametricPath< 2 > PathType;
  
  PathType::Pointer path = PathType::New();
  path->Initialize();

  typedef PathType::ContinuousIndexType    ContinuousIndexType;

  // Create a square
  for(unsigned int i = 0; i < 20; ++i)
    {
    ContinuousIndexType cindex;
    cindex[0] = 0 + drand48();
    cindex[1] = i + drand48();
    path->AddVertex( cindex );
    }
    
  for(unsigned int i = 0; i < 19; ++i)
    {
    ContinuousIndexType cindex;
    cindex[0] = i + drand48();
    cindex[1] = 19 + drand48();
    path->AddVertex( cindex );
    }
   
   for(int i = 18; i >= 0; --i)
    {
    ContinuousIndexType cindex;
    cindex[0] = 19 + drand48();
    cindex[1] = i + drand48();
    path->AddVertex( cindex );
    }
    
  for(int i = 18; i >= 0; --i)
    {
    ContinuousIndexType cindex;
    cindex[0] = i + drand48();
    cindex[1] = 0 + drand48();
    path->AddVertex( cindex );
    }
    
  // Create a vtkPoints object and store the points in it
  vtkSmartPointer<vtkPoints> points =
    vtkSmartPointer<vtkPoints>::New();
 
  const PathType::VertexListType * vertexList = path->GetVertexList ();
  
  std::ofstream fout("output.txt");
 
  for(unsigned int i = 0; i < vertexList->Size(); ++i)
    {
    //std::cout << vertexList->GetElement(i) << std::endl;
    double p[3] = {vertexList->GetElement(i)[0], vertexList->GetElement(i)[1], 0.0};
    fout << vertexList->GetElement(i)[0] << " " << vertexList->GetElement(i)[1] << std::endl;
    points->InsertNextPoint(p);
    }
 fout.close();
 
  // Create a cell array to store the lines in and add the lines to it
  vtkSmartPointer<vtkCellArray> lines =
    vtkSmartPointer<vtkCellArray>::New();
 
  for(unsigned int i = 0; i < points->GetNumberOfPoints(); i++)
    {
    //Create the first line (between Origin and P0)
    vtkSmartPointer<vtkLine> line =
      vtkSmartPointer<vtkLine>::New();
    line->GetPointIds()->SetId(0,i);
    line->GetPointIds()->SetId(1,i+1);
    lines->InsertNextCell(line);
    }
 
  // Create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> linesPolyData =
    vtkSmartPointer<vtkPolyData>::New();
 
  // Add the points to the dataset
  linesPolyData->SetPoints(points);
 
  // Add the lines to the dataset
  linesPolyData->SetLines(lines);
      
  vtkSmartPointer<vtkXMLPolyDataWriter> writer =  
    vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName("output.vtp");
  writer->SetInputConnection(linesPolyData->GetProducerPort());
  writer->Write();
  
  return EXIT_SUCCESS;
}
