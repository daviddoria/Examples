#include <QtConcurrentMap>

#include <iostream>

class MyClass
{
public:
  void Test()
  {
    std::cout << "Test" << std::endl;
  }
};

int main()
{
  QVector<MyClass> v;
  MyClass a;
  v.push_back(a);

  QtConcurrent::blockingMap(v.begin(), v.end(), &MyClass::Test);

  return 0;
}
