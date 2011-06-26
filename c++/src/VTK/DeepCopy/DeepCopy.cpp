#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkPoints> sourcePoints = vtkSmartPointer<vtkPoints>::New();
	
	//add a point
	double sourcePoint[3] = {1.0, 2.0, 3.0};
	sourcePoints->InsertNextPoint(sourcePoint);
			
	//create a polydata to add everything to
    vtkSmartPointer<vtkPolyData> originalPolydata = vtkSmartPointer<vtkPolyData>::New();
	originalPolydata->SetPoints(sourcePoints);

	//output original points
	double origpoint[3];
	originalPolydata->GetPoint(0, origpoint);
	cout << "Original point: (" << origpoint[0] << ", " << origpoint[1] << ", " << origpoint[2] << ")" << endl;
	
    vtkSmartPointer<vtkPolyData> newPolydata = vtkSmartPointer<vtkPolyData>::New();
	//OriginalPolydata->DeepCopy(NewPolydata); //wrong!
	newPolydata->DeepCopy(originalPolydata);
	
	//output new point
	double point[3];
	newPolydata->GetPoint(0, point);
	cout << "Transformed point: (" << point[0] << ", " << point[1] << ", " << point[2] << ")" << endl;
	
	return 0;
}
