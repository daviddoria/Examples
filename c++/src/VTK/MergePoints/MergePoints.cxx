#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkMergePoints.h>
#include <vtkPointSource.h>

int main(int argc, char **argv)
{
  //create a set of points
  vtkSmartPointer<vtkPointSource> pointsSource = 
      vtkSmartPointer<vtkPointSource>::New();
  pointsSource->SetNumberOfPoints(40);
  pointsSource->Update();

  vtkPolyData* points = pointsSource->GetOutput();
  
  //get the first point in the set
  double inSet[3];
  points->GetPoint(0, inSet);
  
  cout << "There are " << points->GetNumberOfPoints() << " input points." << endl;
  
  //insert a point only if it is not already in the set
  vtkSmartPointer<vtkMergePoints> mergePoints = 
      vtkSmartPointer<vtkMergePoints>::New();
  mergePoints->SetDataSet(points);
  mergePoints->InitPointInsertion(points->GetPoints(), points->GetBounds());
  
  int* div = new int[3];
  div = mergePoints->GetDivisions();
  cout << "div: " << div[0] << " " << div[1] << " " << div[2] << endl;
    
  mergePoints->SetDivisions(10,10,10); //segfault without this
  
  int id;
  int inserted = mergePoints->InsertUniquePoint(inSet, id);
  
  cout << "inserted? " << inserted << endl;
  cout << "current id: " << id << endl;
  
  if(inserted == 0)
    {
    cout << "Point already in set!" << endl;
    }
  else if(inserted == 1) //point is not in set
    {
    cout << "Point not already in set, added!" << endl;
    }
  
  cout << "There are now " << points->GetNumberOfPoints() << " points." << endl;
    
  return EXIT_SUCCESS;
}
