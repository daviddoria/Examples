#include <QtConcurrentMap>
#include <QVector>

#include <boost/bind.hpp>

#include <iostream>

class Object{};

class MyClass
{
public:
  float operator()(const Object& object) { return 1.0f;}
  typedef float result_type;
};

int main()
{
  QVector<Object> v;
  Object a;
  v.push_back(a);

  MyClass myClass;

  QVector<float> result = QtConcurrent::blockingMapped<QVector<float> >(v.begin(), v.end(), myClass);

  std::cout << result[0] << std::endl;
  return 0;
}
