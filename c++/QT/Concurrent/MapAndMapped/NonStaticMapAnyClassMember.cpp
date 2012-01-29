#include <QFuture>

#include <QtConcurrentMap>

#include <iostream>
#include <vector>

#include <boost/bind.hpp>

class MyClass
{
public:
  void Test(std::vector<float>& v);
  
  void doubleValue(float& v);
  
};

void MyClass::doubleValue(float& v)
{
  v *= 2.0f;
}


int main()
{
  std::vector<float> v;
  v.push_back(1);
  v.push_back(2);
  v.push_back(3);
  
  MyClass myClass;
  myClass.Test(v);
    
  return 0;
}

void MyClass::Test(std::vector<float>& v)
{
  QtConcurrent::blockingMap<std::vector<float> >(v, boost::bind(&MyClass::doubleValue, this, _1));
  for(unsigned int i = 0; i < v.size(); ++i)
    {
    std::cout << v[i] << std::endl;
    }
}
