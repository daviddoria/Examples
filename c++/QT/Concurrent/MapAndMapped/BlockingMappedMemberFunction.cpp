#include <QtConcurrentMap>
#include <QVector>

#include <iostream>

class MyClass
{
public:
  float Test() const  // Note, this function MUST be const!
  {
    return 1.0f;
  }
};

int main()
{
  QVector<MyClass> v;
  MyClass a;
  v.push_back(a);

  QVector<float> result = QtConcurrent::blockingMapped<QVector<float> >(v.begin(), v.end(), &MyClass::Test);

  return 0;
}
