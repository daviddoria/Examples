#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>

int main(int argc, char *argv[])
{

  ///////// Set Point Normals ///////////
  //create 3 points
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(1.0, 0.0, 0.0);
  points->InsertNextPoint(0.0, 0.0, 0.0);
  points->InsertNextPoint(0.0, 1.0, 0.0);
  
  //add the points to a polydata
  vtkSmartPointer<vtkPolyData> polydata = 
      vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);
  
  //set point normals
  vtkSmartPointer<vtkDoubleArray> pointNormalsArray = 
      vtkSmartPointer<vtkDoubleArray>::New();
  pointNormalsArray->SetNumberOfComponents(3); //3d normals (ie x,y,z)
  pointNormalsArray->SetNumberOfTuples(polydata->GetNumberOfPoints());
  //construct the normal vectors
  double pN1[3] = {1.0, 0.0, 0.0};
  double pN2[3] = {0.0, 1.0, 0.0};
  double pN3[3] = {0.0, 0.0, 1.0};
  //add the data to the normals array
  pointNormalsArray->SetTuple(0, pN1) ;
  pointNormalsArray->SetTuple(1, pN2) ;
  pointNormalsArray->SetTuple(2, pN3) ;
  
  //add the normals to the points in the polydata
  polydata->GetPointData()->SetNormals(pointNormalsArray);

  ///////// Get Point Normals ///////////
  vtkSmartPointer<vtkDoubleArray> pointNormalsRetrieved = 
      vtkDoubleArray::SafeDownCast(polydata->GetPointData()->GetNormals());
  if(pointNormalsRetrieved)
    { 
    cout << "There are " << pointNormalsRetrieved->GetNumberOfTuples() << " point normals." << endl;
  
    for(unsigned int i = 0; i < pointNormalsRetrieved->GetNumberOfTuples(); i++)
      {
      double PN[3];
      pointNormalsRetrieved->GetTuple(i, pN);
      cout << "Point normal " << i << ": " << pN[0] << " " << pN[1] << " " << pN[2] << endl;
      }
    
    }
  else
    {
    cout << "No point normals." << endl;
    }
  
  return EXIT_SUCCESS;
}
