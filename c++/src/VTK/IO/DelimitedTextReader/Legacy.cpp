#include <vtkPolyData.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkDelimitedTextReader.h>
#include <vtkTable.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>

#include <iostream>
#include <string>

int main(int argc, char *argv[])
{
  if(argc != 3)
  {
    std::cout << "Required parameters: InputFilename OutputFilename" << std::endl;
    exit(-1);
  }
  
  std::string InputFilename = argv[1];
  std::string OutputFilename = argv[2];
	
  vtkDelimitedTextReader* Reader = vtkDelimitedTextReader::New();
  Reader->SetFileName(InputFilename.c_str());
  Reader->DetectNumericColumnsOn();
  Reader->Update();
    
  vtkTable* Table = Reader->GetOutput();
    
  vtkDoubleArray* X = vtkDoubleArray::SafeDownCast(Table->GetColumn(0));
  vtkDoubleArray* Y = vtkDoubleArray::SafeDownCast(Table->GetColumn(1));
  vtkDoubleArray* Z = vtkDoubleArray::SafeDownCast(Table->GetColumn(2));
  vtkDoubleArray* Nx = vtkDoubleArray::SafeDownCast(Table->GetColumn(3));
  vtkDoubleArray* Ny = vtkDoubleArray::SafeDownCast(Table->GetColumn(4));
  vtkDoubleArray* Nz = vtkDoubleArray::SafeDownCast(Table->GetColumn(5));
	
    //std::cout << "NumX: " << X->GetNumberOfTuples() << std::endl;
    //std::cout << "NumX: " << X->GetNumberOfValues() << std::endl;
    
  vtkPoints* Points = vtkPoints::New();
  vtkDoubleArray* Normals = vtkDoubleArray::New();
    
  Normals->SetNumberOfComponents(3); //3d normals (ie x,y,z)
    
  for(unsigned int i = 0; i < Table->GetNumberOfRows(); i++)
  {
      //Points->InsertNextPoint(X->GetValue(i), Y[i], Z[i]);
      //Points->InsertNextPoint(X->GetValue(i), Y->GetValue(i), Z->GetValue(i));
      //double x,y,z;
    double x[1];
    double y[1];
    double z[1];
    X->GetTupleValue(i,x);
    Points->InsertNextPoint(x[0],y[0],z[0]);
      
    double n[3];
    n[0] = Nx->GetValue(i);
    n[1] = Ny->GetValue(i);
    n[2] = Nz->GetValue(i);
    Normals->InsertNextTuple(n);
  }
    
  vtkPolyData* polydata = vtkPolyData::New();
  polydata->SetPoints(Points);
  polydata->GetPointData()->SetNormals(Normals);
    
  vtkXMLPolyDataWriter* Writer = vtkXMLPolyDataWriter::New();
  Writer->SetFileName(OutputFilename.c_str());
  Writer->SetInput(polydata);
  Writer->Write();
	
	//cleanup
  Points->Delete();
  Normals->Delete();
  Reader->Delete();
  Writer->Delete();
	
  return 0;
}
