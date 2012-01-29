#include <QtConcurrentMap>
#include <QVector>

#include <boost/bind.hpp>

#include <iostream>

class Object{};

class MyClass
{
public:
  float Test(const Object& object) const { return 1.0f;} // Note, this function MUST be const!
};

int main()
{
  QVector<Object> v;
  Object a;
  v.push_back(a);

  MyClass myClass;

  QVector<float> result = QtConcurrent::blockingMapped<QVector<float> >(v.begin(), v.end(), boost::bind(&MyClass::Test, &myClass, _1));

  std::cout << result[0] << std::endl;
  return 0;
}
