#include <QFuture>

#include <QtConcurrentMap>

#include <iostream>
#include <vector>

class MyClass : public std::vector<float>
{
public:
  void Test();
  void Output();
  
  static void doubleValue(float &v);
  
};

void MyClass::doubleValue(float &v)
{
  v *= 2.0f;
}

int main()
{
  MyClass v;
  v.push_back(1.0);
  v.push_back(2.0);
  v.push_back(3.0);
  
  v.Test();
    
  return 0;
}

void MyClass::Test()
{
  QtConcurrent::blockingMap(*(this), &MyClass::doubleValue);
  Output();

}

void MyClass::Output()
{
  for(unsigned int i = 0; i < this->size(); i++)
    {
    std::cout << (*this)[i] << std::endl;
    }  
}
