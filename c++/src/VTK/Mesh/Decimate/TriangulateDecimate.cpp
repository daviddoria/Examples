#include <vtkPolyData.h>
#include <vtkDecimatePro.h>
#include <vtkSmartPointer.h>
#include <vtkTriangleFilter.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>

int main ( int argc, char *argv[] )
{
  if ( argc != 3 )
  {
    vtkstd::cout << "Required arguments: InputFilename OutputFilename" << vtkstd::endl;
    exit ( -1 );
  }

  vtkstd::string InputFilename = argv[1];
  vtkstd::string OutputFilename = argv[2];

  vtkSmartPointer<vtkXMLPolyDataReader> Reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  Reader->SetFileName(InputFilename.c_str());
  Reader->Update();
    
  vtkPolyData* OriginalMesh = Reader->GetOutput();
	
  vtkstd::cout << "Triangulating..." << vtkstd::endl;
  vtkSmartPointer<vtkTriangleFilter> Triangulate = vtkSmartPointer<vtkTriangleFilter>::New();
  Triangulate->SetInput(OriginalMesh);
  Triangulate->Update();
  vtkPolyData* TriangulatedMesh = Triangulate->GetOutput();
  
  
  vtkstd::cout << "Before decimation" << vtkstd::endl << "------------" << vtkstd::endl;
  std::cout << "There are " << OriginalMesh->GetNumberOfPoints() << " points." << std::endl;
  std::cout << "There are " << OriginalMesh->GetNumberOfPolys() << " triangles." << std::endl;
    
  vtkSmartPointer<vtkDecimatePro> Decimate = vtkSmartPointer<vtkDecimatePro>::New();
  Decimate->SetInput(TriangulatedMesh);
    //Decimate->SetTargetReduction(.99); //99% reduction (if there was 100 triangles, now there will be 1)
  Decimate->SetTargetReduction(.80); //10% reduction (if there was 100 triangles, now there will be 90)
    
  Decimate->Update();
    
  vtkPolyData* DecimatedMesh = Decimate->GetOutput();
    
  vtkstd::cout << "After decimation" << vtkstd::endl << "------------" << vtkstd::endl;

  std::cout << "There are " << DecimatedMesh->GetNumberOfPoints() << " points." << std::endl;
  std::cout << "There are " << DecimatedMesh->GetNumberOfPolys() << " triangles." << std::endl;
  
  vtkSmartPointer<vtkXMLPolyDataWriter> Writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  Writer->SetInput(DecimatedMesh);
  Writer->SetFileName(OutputFilename.c_str());
  Writer->Write();
    
  return 0;
}
