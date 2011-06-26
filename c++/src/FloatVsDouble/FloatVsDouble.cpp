#include <iostream>

void OperateOnDouble(double a);
void OperateOnFloat(float a);

void OperateOnDoublePointer(double* a);
void OperateOnFloatPointer(float* a);

int main(int argc, char *argv[])
{

  double a = 1.1;
  OperateOnDouble(a);
  OperateOnFloat(a);
  
  float b = 2.2;
  OperateOnDouble(b);
  OperateOnFloat(b);
  
  double* c = new double[1];
  c[0] = 3.3;
  OperateOnDoublePointer(c);
  
  OperateOnFloatPointer(reinterpret_cast<float*>(c));
  
  float* d = new float[1];
  d[0] = c[0];
  OperateOnFloatPointer(d);
  
  
	return 0;
}

void OperateOnDouble(double a)
{
  std::cout << a << std::endl;
}

void OperateOnFloat(float a)
{
  std::cout << a << std::endl;
}

void OperateOnDoublePointer(double* a)
{
  std::cout << a[0] << std::endl;
}

void OperateOnFloatPointer(float* a)
{
  std::cout << a[0] << std::endl;
}
