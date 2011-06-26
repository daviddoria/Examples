#include "vtkSmartPointer.h"
#include "vtkTransform.h"

void CreateATransform();
void TransformAPoint();

int main(int argc, char *argv[])
{
  CreateATransform();
	//TransformAPoint();
  return 0;
}

void CreateATransform()
{
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  //std::cout << "Transform: " << *Transform << std::endl;
  
  //Transform->RotateX(90);
  //std::cout << "Transform: " << *Transform << std::endl;
  
  //Transform->RotateY(90);
  //std::cout << "Transform: " << *Transform << std::endl;
  
  //Transform->RotateZ(90);
  //std::cout << "Transform: " << *Transform << std::endl;
  
  //Transform->PreMultiply(); //this is confusing
  transform->PostMultiply(); //this makes the operations happen in sequential order - that is, in the following code, the rotation is applied before the translation
  transform->RotateZ(90);
  transform->Translate(2, 0, 0);
  vtkstd::cout << "Transform: " << *transform << vtkstd::endl;
  
  //std::cout << "concat: " << Transform->GetNumberOfConcatenatedTransforms() << std::endl;
}

void TransformAPoint()
{
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->Translate(1.0, 2.0, 3.0);
  transform->RotateX(90);
  
  double p[3];
  p[0] = 1.0;
  p[1] = 2.0;
  p[2] = 3.0;
  
  double pout[3];
  transform->TransformPoint(p, pout);
  
  vtkstd::cout << pout[0] << " " << pout[1] << " " << pout[2] << vtkstd::endl;

}