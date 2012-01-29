#include <iostream>
#include <vector>

class BaseClass
{
public:
  float GetA() {return A;}
protected:
  float A;
};

template <typename T>
class TestClass : public BaseClass
{
public:
  T GetB();
private:
  T B;
};

int main(int, char *[])
{
  std::vector<BaseClass*> collection;
  //BaseClass* test = new TestClass<float>;
  
  collection.push_back(new TestClass<float>);
  collection.push_back(new TestClass<int>);

  //std::cout <<  << std::endl;
  
  return 0;
}
