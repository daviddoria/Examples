#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkDelaunay2D.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkLookupTable.h>

#include <vtkstd/limits>

int main(int argc, char *argv[])
{
  //created a grid of points (heigh/terrian map)
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
	
  unsigned int GridSize = 10;
  for(unsigned int x = 0; x < GridSize; x++)
    {
    for(unsigned int y = 0; y < GridSize; y++)
      {
      points->InsertNextPoint(x, y, drand48());
      }
    }
  
  //add the grid points to a polydata object
  vtkSmartPointer<vtkPolyData> inputPolyData = 
      vtkSmartPointer<vtkPolyData>::New();
  inputPolyData->SetPoints(points);
	
  //triangulate the grid points
  vtkSmartPointer<vtkDelaunay2D> delaunay = 
      vtkSmartPointer<vtkDelaunay2D>::New();
  delaunay->SetInput(inputPolyData);
  delaunay->Update();
  vtkPolyData* outputPolyData = delaunay->GetOutput();
  
  //find min and max z
  double minz = vtkstd::numeric_limits<double>::infinity();
  double maxz = -1.0 * vtkstd::numeric_limits<double>::infinity();
  
  for(unsigned int i = 0; i < outputPolyData->GetNumberOfPoints(); i++)
    {
    double p[3];
    outputPolyData->GetPoint(i,p);
    if(p[2] < minz)
      {
      minz = p[2];
      }
    if(p[2] > maxz)
      {
      maxz = p[2];
      }
    }
    
  cout << "minz: " << minz << endl;
  cout << "maxz: " << maxz << endl;
  
  //create the color map
  vtkSmartPointer<vtkLookupTable> colorLookupTable = 
      vtkSmartPointer<vtkLookupTable>::New();
  colorLookupTable->SetTableRange(minz, maxz);
  colorLookupTable->Build();
 
  //generate the colors for each point based on the color map
  vtkSmartPointer<vtkUnsignedCharArray> colors = 
      vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents ( 3 );
  colors->SetName ( "Colors" );
  
  cout << "There are " << outputPolyData->GetNumberOfPoints() << " points." << endl;
  
  for(unsigned int i = 0; i < outputPolyData->GetNumberOfPoints(); i++)
    {
    double p[3];
    outputPolyData->GetPoint(i,p);
    
    double dcolor[3];
    colorLookupTable->GetColor(p[2], dcolor);
    cout << "dcolor: " << dcolor[0] << " " << dcolor[1] << " " << dcolor[2] << endl;
    unsigned char color[3];
    for(unsigned int j = 0; j < 3; j++)
      {
      color[j] = 255 * dcolor[j]/1.0;
      }
      cout << "color: " << (int)color[0] << " " << (int)color[1] << " " << (int)color[2] << endl;
    
    colors->InsertNextTupleValue(color);
    }
  
  outputPolyData->GetPointData()->AddArray(colors);
  
  //write the output file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  std::string OutputFile = "Test.vtp";
  writer->SetFileName(OutputFile.c_str());
  writer->SetInput(outputPolyData);
  writer->Write();	
  
  return EXIT_SUCCESS;
}
