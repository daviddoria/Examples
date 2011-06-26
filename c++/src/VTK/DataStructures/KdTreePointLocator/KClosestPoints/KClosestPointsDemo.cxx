#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkIdList.h>
#include <vtkKdTreePointLocator.h>

int main(int argc, char *argv[])
{
  std::cout << "Reading file..." << std::endl;
  
  vtkSmartPointer<vtkXMLPolyDataReader> reader =
      vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(argv[1]);
  reader->Update();

  std::cout << "There are " << reader->GetOutput()->GetNumberOfPoints() << " points." << std::endl;

  std::cout << "Building tree..." << std::endl;
  //Create the tree
  vtkSmartPointer<vtkKdTreePointLocator> pointTree =
      vtkSmartPointer<vtkKdTreePointLocator>::New();
  pointTree->SetDataSet(reader->GetOutput());
  pointTree->BuildLocator();

  std::cout << "Finding K Closest Points..." << std::endl;
  // Find the k closest points to (0,0,0)
  unsigned int k = 3;
  double testPoint[3] = {0.0, 0.0, 0.0};
  vtkSmartPointer<vtkIdList> result =
      vtkSmartPointer<vtkIdList>::New();

  pointTree->FindClosestNPoints(k, testPoint, result);

  for(vtkIdType i = 0; i < k; i++)
    {
    vtkIdType point_ind = result->GetId(i);
    double p[3];
    reader->GetOutput()->GetPoint(point_ind, p);
    std::cout << "Closest point " << i << ": Point "
    << point_ind << ": (" << p[0] << ", " << p[1] << ", " << p[2] << ")" << std::endl;
    }

  return EXIT_SUCCESS;
}

