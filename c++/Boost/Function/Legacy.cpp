#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <iostream>

class MyClass
{
private:
  double Update1()
  {
    std::cout << "Update1" << std::endl;
    return 0.0;
  }
  double Update2()
  {
    std::cout << "Update2" << std::endl;
    return 0.0;
  }

public:
  boost::function< double() > Update;
  void SetUpdateMethod(int method)
  {
    if(method == 1)
    {
      std::cout << "Setting update method to 1..." << std::endl;
      this->Update = boost::bind(&MyClass::Update1,this);
    }
    if(method == 2)
    {
      std::cout << "Setting update method to 2..." << std::endl;
      this->Update = boost::bind(&MyClass::Update2,this);
    }
  }
};

int main ()
{
  MyClass a;
  a.SetUpdateMethod(1);
  a.Update();

  a.SetUpdateMethod(2);
  a.Update();

  return 0;
}
