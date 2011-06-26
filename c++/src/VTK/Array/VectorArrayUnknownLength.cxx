#include <vtkSmartPointer.h>
#include <vtkFloatArray.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkFloatArray> distances = 
      vtkSmartPointer<vtkFloatArray>::New();
  distances->SetNumberOfComponents(3);
  distances->SetName("Distances");
      
  //set values
  for(unsigned int i = 0; i < 5; i++)
    {	
    float tuple[3] = {drand48(), drand48(), drand48()};
    distances->InsertNextTuple(tuple);
    }
  
  cout << "There are " << distances->GetNumberOfTuples() << " tuples." << endl;
  
  //get values
  for(unsigned int i = 0; i < distances->GetNumberOfTuples(); i++)
    {
    double d = distances->GetValue(i);
    cout << d << endl;
    }

  return EXIT_SUCCESS;
}
