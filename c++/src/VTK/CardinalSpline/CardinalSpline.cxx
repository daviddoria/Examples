#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkCardinalSpline.h>
#include <vtkCellArray.h>
#include <vtkColorTransferFunction.h>
#include <vtkContourFilter.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkProperty.h>

int main(int argc, char *argv[])
{
  // Create a set of vertices (polydata)
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(100.0, 0.0, 0.0);
  points->InsertNextPoint(300.0, 0.0, 0.0);

  vtkSmartPointer<vtkCellArray> verts = 
      vtkSmartPointer<vtkCellArray>::New();
  verts->InsertNextCell(2);
  verts->InsertCellPoint(0);
  verts->InsertCellPoint(1);

  // Create a scalar array for the pointdata, each value represents the distance
  // of the vertices from the first vertex
  vtkSmartPointer<vtkFloatArray> distances = 
      vtkSmartPointer<vtkFloatArray>::New();
  distances->SetNumberOfComponents(1);
  distances->SetName("Distances");
  distances->InsertNextValue(100.0);
  distances->InsertNextValue(300.0);

  // Create a cardinal spline to show how to linearly interpolate and contrast
  // the results with the subsequent shepard method
  vtkSmartPointer<vtkCardinalSpline> spline = 
      vtkSmartPointer<vtkCardinalSpline>::New();
  spline->ClosedOff();
  
  // Set the left and right second derivatives to 0 corresponding to linear interpolation
  spline->SetLeftConstraint(2);
  spline->SetLeftValue(0);
  spline->SetRightConstraint(2);
  spline->SetRightValue(0);
  double* r = distances->GetRange();
  double xmin = r[0];
  double xmax = r[1];
  double length = xmax-xmin;
  for( vtkIdType i = 0; i < distances->GetNumberOfTuples(); ++i )
    {
    double x = distances->GetTuple1(i);
    double t = (x - xmin)/length;
    spline->AddPoint( t, x );
    }

  // Evaluate every 50 distance units along the line
  cout << "Spline interpolation:" << endl;
  double dt = .25;
  for( double t = dt ; t <= 1. - dt; t += dt )
    {
    cout << "t: " << t << " value = " << spline->Evaluate( t ) << endl;
    }
 
  return EXIT_SUCCESS;
}
