#include <iostream>
#include <vector>
#include <algorithm>

class TestClass
{
public:
  TestClass();

  std::vector<double>& GetData();

private:
  std::vector<double> Data;

};

bool SpecialSort(const double& value1, const double& value2)
{
  return (value1 < value2);
}

TestClass::TestClass()
{
  this->Data.push_back(0);
  this->Data.push_back(1);
  this->Data.push_back(2);
}

std::vector<double>& TestClass::GetData()
{
  return Data;
}

void OutputVector(const std::vector<double>& v)
{
  for(unsigned int i = 0; i < v.size(); i++)
    {
    std::cout << v[i] << " ";
    }
    
  std::cout << std::endl;
}

int main(int argc, char *argv[])
{
  TestClass test;
  OutputVector(test.GetData());
  
  test.GetData()[1] = 5;

  OutputVector(test.GetData());
  
  //std::sort(test.GetData().begin(), test.GetData().end());
  std::sort(test.GetData().begin(), test.GetData().end(), SpecialSort);
  
  return 0;
}
