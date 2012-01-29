#include <iostream>

void TestUnsignedInt();
void TestFloat();

float modulus(const float a, const float b);

int main(int argc, char *argv[])
{
  TestFloat();
  
  return 0;
}

void TestUnsignedInt()
{
  unsigned int divideBy = 5;
  unsigned int startWith = 21;
  
  std::cout << "21/5 = " << 21/5 << std::endl; // = 4
  std::cout << "21%5 = " << 21%5 << std::endl; // = 1
}

void TestFloat()
{
  float divideBy = 5.0f;
  float startWith = 21.0f;
  
  //std::cout << "21/5 = " << startWith/divideBy << std::endl; // = 4
  //std::cout << "21%5 = " << startWith%divideBy << std::endl; // Can't do this!
  
  std::cout << "21%5 = " << modulus(startWith,divideBy) << std::endl; // = 1
  
}

float modulus(const float a, const float b)
{
  int result = static_cast<int>( a / b );
  return a - static_cast<float>( result ) * b;
}