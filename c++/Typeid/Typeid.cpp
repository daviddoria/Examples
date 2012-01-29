#include <iostream>
#include <typeinfo>

void OutputTypes();
void CheckTypes();

int main(int, char*[])
{
  OutputTypes();
  //CheckTypes();

  return 0;
}

void OutputTypes()
{
  unsigned char uc = 2;
  std::cout << typeid(uc).name() << std::endl; // 'h'

  long long ll = 2;
  std::cout << typeid(ll).name() << std::endl; // 'x'

}

void CheckTypes()
{
  int a = 2;
  double b = 3.0;
  int c = 4.0;

  std::cout << typeid(a).name() << std::endl;
  std::cout << typeid(b).name() << std::endl;

  if(typeid(a) == typeid(b))
    {
    std::cout << "types match." << std::endl;
    }
  else
    {
    std::cout << "types do not match!" << std::endl;
    }

  if(typeid(a) == typeid(c))
    {
    std::cout << "types match." << std::endl;
    }
  else
    {
    std::cout << "types do not match!" << std::endl;
    }

  if(typeid(int) == typeid(int))
    {
    std::cout << "types match." << std::endl;
    }
  else
    {
    std::cout << "types do not match!" << std::endl;
    }
}