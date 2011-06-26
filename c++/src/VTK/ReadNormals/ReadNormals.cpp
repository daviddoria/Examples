#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPolyDataNormals.h>

bool ReadNormals(vtkPolyData* polydata);

int main(int argc, char *argv[])
{
  //Read File
  vtkstd::string filename = argv[1]; //first command line argument
  
  vtkSmartPointer<vtkXMLPolyDataReader> reader = 
      vtkSmartPointer<vtkXMLPolyDataReader>::New();
  cout << "Reading " << filename << endl;
  reader->SetFileName(filename.c_str());
  reader->Update();
  vtkPolyData* polydata = reader->GetOutput();
  
  // Try to read normals directly
  bool HasNormals = ReadNormals(polydata);
  
  if(HasNormals)
    {
    return EXIT_FAILURE;
    }
	  
  // generate normals
  vtkSmartPointer<vtkPolyDataNormals> normalGenerator = 
      vtkSmartPointer<vtkPolyDataNormals>::New();
  
  normalGenerator->SetInput(polydata);
  normalGenerator->Update();
  polydata = normalGenerator->GetOutput();
	  
  /*
  //optional settings
  normalGenerator->SetFeatureAngle(0.1);
  normalGenerator->SetSplitting(1);
  normalGenerator->SetConsistency(0);
  normalGenerator->SetAutoOrientNormals(0);
  normalGenerator->SetComputePointNormals(1);
  normalGenerator->SetComputeCellNormals(0);
  normalGenerator->SetFlipNormals(0);
  normalGenerator->SetNonManifoldTraversal(1);
  */
  
  // Try to read normals again
  ReadNormals(polydata);
  
  return EXIT_SUCCESS;
}

bool ReadNormals(vtkPolyData* polydata)
{
  cout << "Reading normals..." << endl;
  
  //count points
  vtkIdType NumPoints = polydata->GetNumberOfPoints();
  
  //count triangles
  vtkIdType NumPolys = polydata->GetNumberOfPolys();
  
  ////////////////////////////////////////////////////////////////
  //normals in an array
  vtkSmartPointer<vtkDoubleArray> NormalData1 = vtkDoubleArray::SafeDownCast(polydata->GetPointData()->GetArray("Normals"));
  
  if(NormalData1)
    { 
    int nc = NormalData1->GetNumberOfTuples();
    cout << "There are " << nc << " components in NormalData1 (double Point \"Normals\")" << endl;
    return true;
    }
  
  ////////////////////////////////////////////////////////////////
  //double point normals
  vtkSmartPointer<vtkDoubleArray> NormalData2 = vtkDoubleArray::SafeDownCast(polydata->GetPointData()->GetNormals());
  
  if(NormalData2)
    { 
    cout << "There are " << NormalData2->GetNumberOfComponents() << " components in NormalData2 (double Point Normals)" << endl;
    return true;
    }
  
  
  /////////////////////////////////////////////////////////////////////
  //double generic (double) point normals
  vtkSmartPointer<vtkDataArray> NormalData3 = polydata->GetPointData()->GetNormals(); //works
  if(NormalData3)
    { 
    cout << "There are " << NormalData3->GetNumberOfTuples() << " normals in NormalData3 (float Point Normals)" << endl;
    
    //works
    double TestDouble[3];
    NormalData3->GetTuple(0, TestDouble);
    
    cout << "Double: " << TestDouble[0] << " " << TestDouble[1] << " " << TestDouble[2] << endl;
    
    return true;
    }
  
  ///////////////////////////////////////////////////////////////////////////////////
  //double cell normals
  vtkSmartPointer<vtkDoubleArray> NormalData4 = vtkDoubleArray::SafeDownCast(polydata->GetCellData()->GetNormals("cellNormals"));
  
  if(NormalData4)
    { 
    cout << "There are " << NormalData4->GetNumberOfComponents() << " components in NormalData4 (double Cell \"Cell Normals\")" << endl;
    return true;
    }
  
  //if the function has not yet quit, there were none of these types of normals
  cout << "Normals not found!" << endl;
  return false;

}
