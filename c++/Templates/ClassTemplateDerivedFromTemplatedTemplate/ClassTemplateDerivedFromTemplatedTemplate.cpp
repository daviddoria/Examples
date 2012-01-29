#include <iostream>

template<typename T>
class A
{
};

class B
{
};

template< template<class> class T1, class T2>
class C : public T1<T2>
{
public:
  void MyFunction();
};

template< template<class> class T1, class T2>
void C<T1, T2 >::MyFunction()
{

}

int main(int argc, char* argv[])
{
  C<A,B> c;
  c.MyFunction();
  return 0;
}
