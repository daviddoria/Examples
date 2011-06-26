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
  
  cout << "There are " << pointsSource->GetOutput()->GetNumberOfPoints() << " input points." << endl;
  
  /*
  vtkPolyData* points = pointsSource->GetOutput();
  double p[3];
  points->GetPoint(0,p);
  points->GetPoints()->InsertNextPoint(p);
  cout << "There are now " << points->GetNumberOfPoints() << " points." << endl;
  */
  
  vtkSmartPointer<vtkPoints> newPts = 
      vtkSmartPointer<vtkPoints>::New();
  
  //insert a point only if it is not already in the set
  vtkSmartPointer<vtkMergePoints> mergePoints = 
      vtkSmartPointer<vtkMergePoints>::New();
  mergePoints->SetDataSet(points);
  mergePoints->InitPointInsertion(points->GetPoints(), points->GetBounds());
  mergePoints->SetDivisions(10,10,10);
  
  
  //newPts->Allocate(points->GetNumberOfPoints());
  
  {
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
  }
  
  //mergePoints->InitPointInsertion(points->GetPoints(), points->GetBounds());
  //mergePoints->Update();
  
  {
    double r[3] = {1.2, 3.0, 4.5}; // a random point, likely not in the set
    vtkIdType id;
    int inserted = mergePoints->InsertUniquePoint(r, id);

    cout << "inserted? " << inserted << endl;
    cout << "id: " << id << endl;
  
    if(inserted == 0)
    {
      cout << "Point already in set!" << endl;
    
    }
    else if(inserted == 1) //point is not in set
    {
      cout << "Point not already in set, added!" << endl;
    }
  }
   

  cout << "There are " << points->GetNumberOfPoints() << " points after the merge." << endl;
  
  return EXIT_SUCCESS;
}
