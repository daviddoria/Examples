#include <QFuture>
#include <QVector>
#include <QtConcurrentMap>

#include <iostream>
#include <vector>

#include <boost/bind.hpp>

class Object{};

class MyClass
{
public:
  float Test(const Object& object) {return 1.0;}
};

int main()
{
  std::vector<Object> objects;
  Object a;
  objects.push_back(a);
  //QVector<float> differences = QtConcurrent::blockingMap(objects.begin(), objects.end(), &MyClass::Test, _1);
  //QVector<float> differences = QtConcurrent::blockingMapped(objects.begin(), objects.end(), &MyClass::TestNoParams);

  return 0;
}
