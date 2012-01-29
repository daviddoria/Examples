#include <iostream>

class MyClass
{
public:
  
  template<typename T>
  void DoSomething(){}
};

template<typename A>
void MyFunction(A a)
{
  //a.DoSomething<A>(); // error: expected primary-expression before '>' token
  
  a.template DoSomething<A>();
}


int main()
{
  MyClass a;
  MyFunction<MyClass>(a);
  return 0;
}