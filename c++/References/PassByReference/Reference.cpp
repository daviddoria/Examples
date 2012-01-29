#include <iostream>

class TestClass
{
public:
  
  TestClass(float& value) : Value(value)
  {
    
  }
  
  void SetValue(float& value)
  {
    this->Value = value;
  }
  
  float& Value;
};

int main(int, char *[])
{
  float a = 2.0;
  TestClass test(a);
  
  std::cout << "a: " << a << std::endl;
  std::cout << "test.Value: " << test.Value << std::endl;
  
  float b = 3.0;
  test.SetValue(b);
  
  std::cout << "a: " << a << std::endl;
  std::cout << "b: " << b << std::endl;
  std::cout << "test.Value: " << test.Value << std::endl;
  return 0;
}
