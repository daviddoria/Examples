#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkDelaunay3D.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include <vtkCellArray.h>
#include <vtkDataSetSurfaceFilter.h>

#include <vector>
#include <algorithm>

int main(int argc, char **argv)
{ 
  //parse command line arguments
  if(argc != 2)
    {
    cout << "Required arguments: Filename" << endl;
    return EXIT_FAILURE;
    }
    
  vtkstd::string inputFilename = argv[1];
 
  vtkSmartPointer<vtkXMLPolyDataReader> reader = 
      vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(inputFilename.c_str());
  reader->Update();

  //randomize the points
  std::vector<int> v;
  
  //add 10 integers (0 to 9)
  for(vtkIdType i = 0; i < reader->GetOutput()->GetNumberOfPoints(); i++)
    {
    v.push_back(i);
    }
    
  std::random_shuffle(v.begin(), v.end());
  
  //create a new point set with the points in the randomized order
  vtkSmartPointer<vtkPoints> shuffledPoints =
      vtkSmartPointer<vtkPoints>::New();
  for(int i = 0; i < v.size(); i++)
    {
    shuffledPoints->InsertNextPoint(reader->GetOutput()->GetPoint(v[i]));
    }
  
  vtkSmartPointer<vtkPolyData> shuffledPolyData = 
      vtkSmartPointer<vtkPolyData>::New();
  shuffledPolyData->SetPoints(shuffledPoints);
  
  // Create the convex hull of the pointcloud
  vtkSmartPointer<vtkDelaunay3D> delaunay = 
      vtkSmartPointer< vtkDelaunay3D >::New();
  delaunay->SetInputConnection(shuffledPolyData->GetProducerPort());
  delaunay->Update();
  
  vtkSmartPointer<vtkXMLUnstructuredGridWriter> ugWriter = 
      vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
  ugWriter->SetInputConnection(delaunay->GetOutputPort());
  ugWriter->SetFileName("delaunay.vtu");
  ugWriter->Write();
  
  vtkSmartPointer<vtkDataSetSurfaceFilter> surfaceFilter = 
      vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
  surfaceFilter->SetInputConnection(delaunay->GetOutputPort());
  surfaceFilter->Update();  
 
  vtkSmartPointer<vtkXMLPolyDataWriter> outputWriter = 
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  outputWriter->SetFileName("output.vtp");
  outputWriter->SetInput(surfaceFilter->GetOutput());
  outputWriter->Write();
  
  /*
  // Create the convex hull of the pointcloud
  vtkSmartPointer<vtkDelaunay3D> delaunay = 
      vtkSmartPointer< vtkDelaunay3D >::New();
  delaunay->SetInput(reader->GetOutput());
  delaunay->Update();
  
  vtkSmartPointer<vtkXMLUnstructuredGridWriter> ugWriter = 
      vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
  ugWriter->SetInputConnection(delaunay->GetOutputPort());
  ugWriter->SetFileName("delaunay.vtu");
  ugWriter->Write();
  
  vtkSmartPointer<vtkDataSetSurfaceFilter> surfaceFilter = 
      vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
  surfaceFilter->SetInputConnection(delaunay->GetOutputPort());
  surfaceFilter->Update();  
 
  vtkSmartPointer<vtkXMLPolyDataWriter> outputWriter = 
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  outputWriter->SetFileName("output.vtp");
  outputWriter->SetInput(surfaceFilter->GetOutput());
  outputWriter->Write();
  */
  
  return EXIT_SUCCESS;
}