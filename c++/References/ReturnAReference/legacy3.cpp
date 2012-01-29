#include <iostream>
#include <vector>
#include <algorithm>

class TestClass
{
public:
  TestClass();

  std::vector<std::vector<double> >& GetData();

private:
  std::vector<std::vector<double> > Data;

};

bool SpecialSort(const double& value1, const double& value2)
{
  return (value1 < value2);
}

TestClass::TestClass()
{
  std::vector<double> a;
  a.push_back(0);
  a.push_back(1);
  a.push_back(2);

  this->Data.push_back(a);
  this->Data.push_back(a);
  this->Data.push_back(a);
}

std::vector<std::vector<double> >& TestClass::GetData()
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
//   OutputVector(test.GetData());
//
//   test.GetData()[1] = 5;
//
//   OutputVector(test.GetData());

  //std::sort(test.GetData().begin(), test.GetData().end());
  //std::sort(test.GetData().begin(), test.GetData().end(), SpecialSort);

  std::sort(test.GetData()[0].begin(), test.GetData()[0].end(), SpecialSort);

  return 0;
}
