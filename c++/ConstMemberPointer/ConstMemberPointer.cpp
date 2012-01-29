#include <iostream>

class MyClass
{
public:

 MyClass() : A(NULL) {}
 const int* const A; // This has to be initialized because it cannot be assigned.
};

class MyClass2
{
public:

 const int* A; // this is ok, because the pointer is not const, it is just a pointer that doesn't allow you to change the values of the data it points to
};

int main()
{
 MyClass myClass;
 MyClass2 myClass2;
 
 return 0;
}