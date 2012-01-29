#include <iostream>

class MyClass
{
public:

  static void ConstPointerToConst(const int* const a){}

  static void PointerToConst(const int* a){}

  static void ConstPointer(int* const a){}
};

int main()
{
  const int* const constPointerToConst(new int);

  const int* pointerToConst;

  int* const constPointer(new int);

  MyClass::ConstPointerToConst(pointerToConst);
  MyClass::ConstPointerToConst(constPointer);
  MyClass::ConstPointerToConst(constPointerToConst);

  MyClass::PointerToConst(pointerToConst);
  MyClass::PointerToConst(constPointer);
  MyClass::PointerToConst(constPointerToConst);

  //MyClass::ConstPointer(pointerToConst);
  MyClass::ConstPointer(constPointer);
  //MyClass::ConstPointer(constPointerToConst);
 
 return 0;
}