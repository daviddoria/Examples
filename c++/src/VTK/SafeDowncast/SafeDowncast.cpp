#include <vtkDoubleArray.h>
#include <vtkDataArray.h>

#include <iostream>

int main(int argc, char *argv[])
{
  vtkDataArray* Array;
  //vtkDoubleArray* DoubleArray = vtkDoubleArray::New();
  vtkDoubleArray* DoubleArray = vtkDoubleArray::SafeDownCast(Array);
  /*
  vtkDoubleArray* DoubleDistances = vtkDoubleArray::SafeDownCast(polydata->GetPointData()->GetArray("Distances"));

	vtkFloatArray* Distances = vtkFloatArray::New();
	Distances->SetNumberOfComponents(1);
	Distances->SetName("Distances");
 
	for(unsigned int i = 0; i < 20; i++)
	{	
		Distances->InsertNextValue(drand48());
	}
*/
	return 0;
}
