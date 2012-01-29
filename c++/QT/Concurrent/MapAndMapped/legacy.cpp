#include <QFuture>

#include <QtConcurrentMap>

#include <iostream>
#include <vector>

#include <boost/bind.hpp>

struct MyType
{
  float value;

  float Double();
  void Triple();
};

float MyType::Double()
{
  return this->value * 2.0;
}

class MyClass
{
public:
  void Test(std::vector<MyType>& v);

};

int main()
{
  std::vector<MyType> v;
  MyType a;
  a.value = 1.0;
  v.push_back(a);
  a.value = 2.0;
  v.push_back(a);
  a.value = 3.0;
  v.push_back(a);

  MyClass myClass;
  myClass.Test(v);

  return 0;
}

void MyClass::Test(std::vector<MyType>& v)
{

  //std::vector<float> result = QtConcurrent::blockingMapped<std::vector<float> >(v, &MyType::Double);
  //std::vector<float> result = QtConcurrent::blockingMapped(v, &MyType::Double);
  for(unsigned int i = 0; i < result.size(); ++i)
    {
    std::cout << result[i] << std::endl;
    }

}
