#include <vtksys/ios/sstream>

#include <vtkSmartPointer.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataWriter.h>

/* Example file:
1 2 3
4 5 6 
7 8 9
*/

int main(int argc, char *argv[])
{		
  //parse command line arguments
  if(argc != 2)
    {
    vtkstd::cout << "Required arguments: Filename" << vtkstd::endl;
    exit(-1);
    }
    
  vtkstd::string Filename = argv[1];	
  vtkstd::ifstream fin(Filename.c_str());

  //get all data from the file
  vtkstd::string line;
  vtkSmartPointer<vtkPoints> Points = vtkSmartPointer<vtkPoints>::New();

  while(getline(fin, line))
    {
    double x,y,z;
    vtkstd::stringstream linestream;
    linestream << line;
    linestream >> x >> y >> z;
    
    Points->InsertNextPoint(x, y, z);
    }
  
  fin.close();

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

  polydata->SetPoints(Points);

  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName("Points.vtp");
  writer->SetInput(polydata);
  writer->Write();	

  return 0;
}

