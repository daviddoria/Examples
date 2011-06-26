#include <vtkSmartPointer.h>
#include <vtkMatrix3x3.h>
	
void OutputVector(double* a);
void OutputMatrixPointer(double a[9]);

void Transpose();

void Invert();

void Construct();
void Retrieve();

void MultiplyPoint();
void MultiplyPointInPlace();

void MultiplyMatrix();
void MultiplyMatrixInPlace();

void MultiplyMatrixPointers();

void Copy();
void Identity();
void Print();
void ElementLocation();

int main(int argc, char *argv[])
{
  ElementLocation();
  //Transpose();
  //Invert();
  //Construct();
  //Retrieve();
  
  //MultiplyPoint();
  //MultiplyPointInPlace();
  
  //MultiplyMatrix();
  //MultiplyMatrixInPlace();
  
  //MultiplyMatrixPointers();
  
  //Copy();
  //Identity();
  //Print();
  
  return EXIT_SUCCESS;
}

void ElementLocation()
{
  vtkSmartPointer<vtkMatrix3x3> M = vtkSmartPointer<vtkMatrix3x3>::New();
  M->SetElement(1,0, 5);
  cout << *M << endl;
}

void Print()
{
  vtkSmartPointer<vtkMatrix3x3> M = vtkSmartPointer<vtkMatrix3x3>::New();
  
  cout << *M << endl;
}

void Construct()
{
  vtkSmartPointer<vtkMatrix3x3> M = vtkSmartPointer<vtkMatrix3x3>::New();
  
  cout << *M << endl;
  
  M->SetElement(0,0,2.0); //set element (0,0) to 1.0
  
  double test = M->GetElement(0,0);
  
  cout << "Test: " << test << endl;
}

void Retrieve()
{
  vtkSmartPointer<vtkMatrix3x3> M = vtkSmartPointer<vtkMatrix3x3>::New();
  
  cout << *M << endl;
  
  M->SetElement(0,0,2.0); //set element (0,0) to 1.0
  
  double test = M->GetElement(0,0);
  
  cout << "Test: " << test << endl;
}

void Invert()
{
  vtkSmartPointer<vtkMatrix3x3> M = vtkSmartPointer<vtkMatrix3x3>::New();
  
  M->SetElement(1,0,2.0); //set element (0,0) to 1.0
  
  cout << *M << endl;
  
  M->Invert();
  
  cout << *M << endl;

}

void Transpose()
{
  vtkSmartPointer<vtkMatrix3x3> M = vtkSmartPointer<vtkMatrix3x3>::New();
  
  M->SetElement(2,1,2.0); //set element (0,0) to 1.0
  
  cout << *M << endl;
  
  M->Transpose();
  
  cout << *M << endl;
  
}

void MultiplyPoint()
{
  vtkSmartPointer<vtkMatrix3x3> M = vtkSmartPointer<vtkMatrix3x3>::New();
  
  M->SetElement(0,0,2.0); //set element (0,0) to 1.0

  double pin[3] = {1.0, 2.0, 3.0};
  cout << "Pin: " << endl;
  OutputVector(pin);
  
  double pout[3];
  
  M->MultiplyPoint(pin, pout);
  cout << "Pout: " << endl;
  OutputVector(pout);
  
  //result should be [2;2;3]
  
}

void MultiplyPointInPlace()
{
  
  vtkSmartPointer<vtkMatrix3x3> M = vtkSmartPointer<vtkMatrix3x3>::New();
  
  M->SetElement(0,0,2.0); //set element (0,0) to 1.0

  double pin[3] = {1.0, 2.0, 3.0};
  cout << "Pin: " << endl;
  OutputVector(pin);
  
  double pout[3];

  M->MultiplyPoint(pin, pin);
  cout << "Pin: " << endl;
  OutputVector(pin);
  //result should be [2;2;3]
}

void MultiplyMatrixPointers()
{
  
  vtkstd::cout << " MultiplyMatrixPointers() " << vtkstd::endl;
  double a[9];
  //row major?
  a[0] = 1;
  a[1] = 0;
  a[2] = 0;
  a[3] = 4;
  a[4] = 1;
  a[5] = 0;
  a[6] = 0;
  a[7] = 0;
  a[8] = 1;
  
  cout << "A: " << endl;
  OutputMatrixPointer(a);
  /*
  1 0 0
  4 1 0
  0 0 1
  */
  
  
  
  double b[9];
  b[0] = 1;
  b[1] = 0;
  b[2] = 0;
  b[3] = 0;
  b[4] = 1;
  b[5] = 0;
  b[6] = 5;
  b[7] = 2;
  b[8] = 1;
  
  cout << "B: " << endl;
  OutputMatrixPointer(b);
  
  /*
  {1, 0, 0};
  {0, 1, 0};
  {5, 2, 1};
  */
  
  double c[9];
  
  vtkMatrix3x3::Multiply3x3(a,b,c);
  
  cout << "c: " << endl;
  OutputMatrixPointer(c);
  
  //result of A*B should be [1 0 0; 4 1 0; 5 2 1]
}

void MultiplyMatrix()
{
  vtkSmartPointer<vtkMatrix3x3> A = vtkSmartPointer<vtkMatrix3x3>::New();
  A->SetElement(1,0,4.0);
  
  cout << "A: " << *A << endl;
  
  vtkSmartPointer<vtkMatrix3x3> B = vtkSmartPointer<vtkMatrix3x3>::New();
  B->SetElement(2,0,5.0);
  B->SetElement(2,1,2.0);
  
  cout << "B: " << *B << endl;
  
  vtkSmartPointer<vtkMatrix3x3> C = vtkSmartPointer<vtkMatrix3x3>::New();
  
  vtkMatrix3x3::Multiply3x3(A,B,C);
  
  cout << "C: " << *C << endl;
  
  //result of A*B should be [1 0 0; 4 1 0; 5 2 1]
}

void MultiplyMatrixInPlace()
{
  vtkSmartPointer<vtkMatrix3x3> A = vtkSmartPointer<vtkMatrix3x3>::New();
  A->SetElement(1,0,4.0);
  
  cout << "A: " << *A << endl;
  
  vtkSmartPointer<vtkMatrix3x3> B = vtkSmartPointer<vtkMatrix3x3>::New();
  B->SetElement(2,0,5.0);
  B->SetElement(2,1,2.0);
  
  cout << "B: " << *B << endl;
  
  vtkMatrix3x3::Multiply3x3(A,B,A);
  
  cout << "A: " << *A << endl;
  //result of A*B should be [1 0 0; 4 1 0; 5 2 1]
}

void Copy()
{
  vtkSmartPointer<vtkMatrix3x3> M1 = vtkSmartPointer<vtkMatrix3x3>::New();
  vtkSmartPointer<vtkMatrix3x3> M2 = vtkSmartPointer<vtkMatrix3x3>::New();
  
  M1->SetElement(0,0, 4.0);
  cout << "M1: " << *M1 << endl;
  
  cout << "M2: " << *M2 << endl;
  
  M2->DeepCopy(M1);
  
  cout << "M2: " << *M2 << endl;
  
}

void Identity()
{
  vtkSmartPointer<vtkMatrix3x3> M = vtkSmartPointer<vtkMatrix3x3>::New();
  M->SetElement(0,0,5.0);
  cout << "M: " << endl;
  
  M->Identity();
  
  cout << "M: " << endl;
  
}

void OutputVector(double* a)
{
  for(unsigned int i = 0; i < 3; i++)
    {
    cout << a[i] << " ";
    }
  
  cout << endl;
}

void OutputMatrixPointer(double a[9])
{
  cout << a[0] << " " << a[1] << " " << a[2] << endl;
  cout << a[3] << " " << a[4] << " " << a[5] << endl;
  cout << a[6] << " " << a[7] << " " << a[8] << endl;

  cout << endl;
}