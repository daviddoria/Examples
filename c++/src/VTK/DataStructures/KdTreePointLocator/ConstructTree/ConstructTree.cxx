#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkIdList.h>
#include <vtkKdTree.h>
#include <vtkDataSetCollection.h>

int main(int argc, char *argv[])
{
  //Setup point coordinates
  double x[3] = {1.0, 0.0, 0.0};
  double y[3] = {0.0, 1.0, 0.0};
  double z[3] = {0.0, 0.0, 1.0};

  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> vertices = 
      vtkSmartPointer<vtkCellArray>::New();

  for ( unsigned int i = 0; i < 3; ++i )
    {
    //Declare a variable to store the index of the point that gets added. This behaves just like an unsigned int.
    vtkIdType pid[1];

    //Add a point to the polydata and save its index, which we will use to create the vertex on that point.
    pid[0] = points->InsertNextPoint ( x[i], y[i], z[i] );

    //create a vertex cell on the point that was just added.
    vertices->InsertNextCell ( 1,pid );
    }
  
  cout << "There are " << points->GetNumberOfPoints() << " points." << endl;
  cout << "There are " << vertices->GetNumberOfCells() << " cells." << endl;
  
  vtkSmartPointer<vtkPolyData> polydata = 
      vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);
  polydata->SetVerts(vertices);
  
  //Create the tree
  vtkSmartPointer<vtkKdTree> kDTree = 
      vtkSmartPointer<vtkKdTree>::New();
  kDTree->AddDataSet(polydata);
  kDTree->BuildLocator();

  cout << "numberOfRegionsOrLess: " << kDTree->GetNumberOfRegionsOrLess() << endl;
  cout << "numberOfRegionsOrMore: " << kDTree->GetNumberOfRegionsOrMore() << endl;
  //this doesn't work (why?)
  //vtkstd::cout << "Number of points in tree: " << KDTree->GetDataSets()->GetNextDataSet()->GetNumberOfPoints() << vtkstd::endl;
  
  cout << "Number of points in tree: " << kDTree->GetDataSet(0)->GetNumberOfPoints() << endl;
  
  double p[3];
  kDTree->GetDataSet(0)->GetPoint(0,p);
  cout << "p: " << p[0] << " " << p[1] << " " << p[2] << endl;

  float* centers = kDTree->ComputeCellCenters();
  
  for(unsigned int i = 0; i < kDTree->GetNumberOfCells(); i++)
    {
    cout << centers[0 + i*3] << " , " << centers[1 + i*3] << " , " << centers[2 + i*3] << endl;
    }
  return EXIT_SUCCESS;
}

