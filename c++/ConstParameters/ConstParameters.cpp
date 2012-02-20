#include <iostream>

class MyClass
{
public:

  static void ConstPointerToConst(const int* const a){}

  static void PointerToConst(const int* a){}

  static void ConstPointer(int* const a){}
  
  // static void ConstArray(int[] const a){} // Not a good way to pass an array as const. You can pass a const reference to the array, but it is dangerous and therefore not recommended. Just pass a pointer instead.
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