#include <iostream>

void StaticCast();
void UnsignedCharCast();

int main(int, char *[])
{
  //StaticCast();
  UnsignedCharCast();

  return 0;
}

void StaticCast()
{
  double a = 3.4;

  std::cout << "double a: " << a << std::endl;
  std::cout << "int a: " << (int)a << std::endl;
  std::cout << "int a: " << int(a) << std::endl;
  std::cout << "int a: " << static_cast<int> (a) << std::endl;

  double b = 0.4;
  unsigned char buc = static_cast<unsigned char>(b);
  std::cout << "b: " << b << " buc: " << static_cast<unsigned int>(buc) << std::endl;

}

void UnsignedCharCast()
{
  {
  unsigned char a = 100;
  unsigned char b = 105;

  unsigned char c = (a+b)/2;
  std::cout << (int)c << std::endl;
  }

  {
  unsigned char a = 200;
  unsigned char b = 205;

  unsigned char c = (a+b)/2;
  std::cout << (int)c << std::endl;
  }

  {
  unsigned char a = 180;
  unsigned char b = 185;

  unsigned char c = (a+b)/2;
  std::cout << (int)c << std::endl;
  }

  {
  double a = 187.3;
  unsigned char b = (unsigned char)a;
  std::cout << (int)b << std::endl;
  }

  {
  double a = 187.3;
  unsigned char b = static_cast<unsigned char>(a);
  std::cout << (int)b << std::endl;
  }
}