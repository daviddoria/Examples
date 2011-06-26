#include <vtkSmartPointer.h>
#include <vtkFloatArray.h>
#include <vtkIntArray.h>

void OneD();
void ThreeD();
void IntArray1D();
void ThreeDKnownLength();

int main(int, char *[])
{
  //OneD();
  //ThreeD();
  //IntArray1D();
  ThreeDKnownLength();
  return 0;
}

void IntArray1D()
{
  vtkSmartPointer<vtkIntArray> ints = 
    vtkSmartPointer<vtkIntArray>::New();
  ints->SetNumberOfComponents(1);
  
  ints->InsertNextValue(5);

  for(unsigned int i = 0; i < ints->GetNumberOfTuples(); i++)
    {
    int val = ints->GetValue(i);
    std::cout << val << std::endl;
    }
}

void OneD()
{
  vtkSmartPointer<vtkFloatArray> distances = 
    vtkSmartPointer<vtkFloatArray>::New();
  distances->SetNumberOfComponents(1);
  distances->SetName("Distances");

  // Set values
  for(unsigned int i = 0; i < 20; i++)
    {
    distances->InsertNextValue(drand48());
    }
  
  std::cout << "There are " << distances->GetNumberOfTuples() << " values." << std::endl;
  
  // Get values
  for(unsigned int i = 0; i < distances->GetNumberOfTuples(); i++)
    {
    double d = distances->GetValue(i);
    std::cout << d << std::endl;
    }
}

void ThreeD()
{
  vtkSmartPointer<vtkFloatArray> v = 
    vtkSmartPointer<vtkFloatArray>::New();
  v->SetNumberOfComponents(3);
  
  // Set values
  for(unsigned int i = 0; i < 5; i++)
    {
    float temp[3] = {drand48(), drand48(), drand48()};
    v->InsertNextTupleValue(temp);
    std::cout << temp[0] << " " << temp[1] << " " << temp[2] << std::endl;
    }
  
  std::cout << "There are " << v->GetNumberOfTuples() << " values." << std::endl;
  
  // Get values
  for(unsigned int i = 0; i < v->GetNumberOfTuples(); i++)
    {
    float temp[3];
    v->GetTupleValue(i, temp);
    std::cout << temp[0] << " " << temp[1] << " " << temp[2] << std::endl;
    }
}


void ThreeDKnownLength()
{
  vtkSmartPointer<vtkFloatArray> v =
    vtkSmartPointer<vtkFloatArray>::New();
  v->SetNumberOfComponents(3);
  v->SetNumberOfTuples(5);

  // Set values
  for(unsigned int i = 0; i < v->GetNumberOfTuples(); i++)
    {
    float temp[3] = {drand48(), drand48(), drand48()};
    v->SetTupleValue(i, temp);
    std::cout << temp[0] << " " << temp[1] << " " << temp[2] << std::endl;
    }

  std::cout << "There are " << v->GetNumberOfTuples() << " values." << std::endl;

  // Get values
  for(unsigned int i = 0; i < v->GetNumberOfTuples(); i++)
    {
    float temp[3];
    v->GetTupleValue(i, temp);
    std::cout << temp[0] << " " << temp[1] << " " << temp[2] << std::endl;
    }
}
