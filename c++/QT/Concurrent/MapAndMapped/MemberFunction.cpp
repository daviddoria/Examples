#include <QFuture>

#include <QtConcurrentMap>

#include <iostream>
#include <vector>

class MyClass
{
public:
  void Test(std::vector<float>& v);
  
  static void doubleValue(float &v);
  
};

void MyClass::doubleValue(float &v)
{
  v *= 2.0f;
}

int main()
{
  std::vector<float> v;
  v.push_back(1.0);
  v.push_back(2.0);
  v.push_back(3.0);
  
  MyClass myClass;
  myClass.Test(v);
    
  return 0;
}

void MyClass::Test(std::vector<float>& v)
{
  QtConcurrent::blockingMap(v, &MyClass::doubleValue);

}
