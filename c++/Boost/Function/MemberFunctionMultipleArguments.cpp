#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <iostream>

class MyClass
{
private:
  double Update1(int val1, int val2)
  {
    std::cout << "Update1 " << val1 << " " << val2 << std::endl;
  }
  double Update2(int val1, int val2)
  {
    std::cout << "Update2 " << val1 << " " << val2 << std::endl;
  }

public:
  boost::function< double(int, int) > Update;
  
  void SetUpdateMethod(int method)
  {
    if(method == 1)
    {
      std::cout << "Setting update method to 1..." << std::endl;
      this->Update = boost::bind(&MyClass::Update1,this,_1,_2);
    }
    if(method == 2)
    {
      std::cout << "Setting update method to 2..." << std::endl;
      this->Update = boost::bind(&MyClass::Update2,this,_1,_2);
    }
  }
};
  
int main () 
{
  MyClass a;
  a.SetUpdateMethod(1);
  a.Update(3,4);

  a.SetUpdateMethod(2);
  a.Update(5,6);
  
  return 0;
}
