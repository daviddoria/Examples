#include <vtkMinimalStandardRandomSequence.h>
#include <vtkSmartPointer.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkMinimalStandardRandomSequence> sequence = 
      vtkSmartPointer<vtkMinimalStandardRandomSequence>::New();
  
  sequence->SetSeed(1);  // initialize the sequence
  double x=sequence->GetValue(); // or seq->GetRangeValue(-1.0,1.0); in your case
  sequence->Next();
  double y=sequence->GetValue();
  sequence->Next();
  double z=sequence->GetValue();
  
  vtkstd::cout << "x: " << x << " y: " << y << " z: " << z << vtkstd::endl;
  
	return 0;
}
