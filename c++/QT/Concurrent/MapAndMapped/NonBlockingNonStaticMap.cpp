#include <QFuture>

#include <QtConcurrentMap>

#include <iostream>
#include <vector>

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

void MyType::Triple()
{
  this->value *= 2.0;
}


class MyClass
{
public:
  void Test(std::vector<MyType>& v);
  
  float doubleValue(const float v);
  
};

float MyClass::doubleValue(const float v)
{
  return v * 2.0f;
}


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

//   std::vector<MyType> result = QtConcurrent::blockingMapped<std::vector<MyType> >(v, &MyType::Double);
//   for(unsigned int i = 0; i < result.size(); ++i)
//     {
//     std::cout << result[i].value << std::endl;
//     }

  QtConcurrent::blockingMap<std::vector<MyType> >(v, &MyType::Triple);
  for(unsigned int i = 0; i < v.size(); ++i)
    {
    std::cout << v[i].value << std::endl;
    }
}
