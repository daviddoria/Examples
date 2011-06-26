#include <iostream>
#include <fstream>

//VNL
#include <vnl/vnl_vector.h>
#include <vnl/vnl_double_2.h>
#include <vnl/vnl_double_3.h>

void TestVector();
void TestFixedSizeVector();
void TestScalarMultiply();
void TestAdd();
void TestNorm();
void TestProjectVector2D();
bool ProjectVector2D(const vnl_vector<double> a, const vnl_vector<double> b, vnl_vector<double> &projection);

int main()
{
  //TestVector();
  //TestFixedSizeVector();
  //TestScalarMultiply();
  //TestNorm();
  //TestAdd();
  TestProjectVector2D();
  
  return 0;
}

void TestNorm()
{
  vnl_vector<double> x(3);
  x(0) = 10;
  x(1) = 20;
  x(2) = 30;
  
  std::cout << x.two_norm() << std::endl;
  std::cout << x.magnitude() << std::endl;
  
}

void TestAdd()
{
  vnl_vector<double> x(3);
  x(0) = 10;
  x(1) = 20;
  x(2) = 30;
  
  //std::cout << x + 1 << std::endl;
  //std::cout << 1 + x << std::endl; // no match for operator+
  
  
}

void TestVector()
{
  vnl_vector<double> x(3);
  x(0) = 10;
  x(1) = 20;
  x(2) = 30;
  
  std::cout << "x = " << x << std::endl;
	
}

void TestScalarMultiply()
{
  vnl_vector<double> x(3);
  x(0) = 10;
  x(1) = 20;
  x(2) = 30;
  
  std::cout << "x = " << x << std::endl;
  
  x *= 2;
  std::cout << "x = " << x << std::endl;
}

void TestFixedSizeVector()
{
  vnl_double_3 a(1,2,3);
  std::cout << a << std::endl;
  
  vnl_double_3 b(4,5,6);

  double dot = dot_product(a,b);

  std::cout << "Dot: " << dot << std::endl;

}

void TestProjectVector2D()
{

  vnl_vector<double> a(2);
  a(0) = 2;
  a(1) = -5;
  vnl_vector<double> b(2);
  b(0) = 5;
  b(1) = 1;
  vnl_vector<double> projection(2);
  vnl_vector<double> correct(2);
  correct(0) = 25./26.;
  correct(1) = 5./26.;
  ProjectVector2D(a,b,projection);

  std::cout << "Correct: " << correct << std::endl << "Actual: " << projection << std::endl;
}

bool ProjectVector2D(const vnl_vector<double> a, const vnl_vector<double> b, vnl_vector<double> &projection)
{
  bool invalid = false;
  if(a.size() != b.size())
    {
    invalid = true;
    }
    
  double bSquared = dot_product(b,b);

  if(bSquared == 0)
    {
    invalid = true;
    }

  if(invalid)
    {
    projection(0) = 0;
    projection(1) = 0;
    return false;
    }

  double scale = dot_product(a,b)/bSquared;

  for(unsigned int i = 0; i < a.size(); i++)
    {
    projection(i) = scale * b(i);
    }
  
  return true;
}