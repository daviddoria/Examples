#include <vtkSmartPointer.h>
#include <vtkFloatArray.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkFloatArray> distances = 
    vtkSmartPointer<vtkFloatArray>::New();
  distances->SetNumberOfComponents(3);
  distances->SetNumberOfTuples(5);
  distances->SetName("Distances");

  //set values
  std::cout << "Set values: " << std::endl;
  for(unsigned int i = 0; i < distances->GetNumberOfTuples(); i++)
    {	
    float tuple[3] = {(float)drand48(), (float)drand48(), (float)drand48()};
    std::cout << tuple[0] << " " << tuple[1] << " " << tuple[2] << std::endl;
    distances->SetTuple(i, tuple);
    }
  
  // Get values
  std::cout << "Get values: " << std::endl;
  for(unsigned int i = 0; i < distances->GetNumberOfTuples(); i++)
    {
    //float tuple[3];
    float* tuple;
    tuple = Distances->GetTuple(i);
    std::cout << tuple[0] << " " << tuple[1] << " " << tuple[2] << std::endl;
    }

  
  return EXIT_SUCCESS;
}
