#include <iostream>
#include <vector>
#include <boost/graph/graph_concepts.hpp>

class TestClass
{
public:
  TestClass();

  void OutputData();

  double& GetElement();

private:
  std::vector<double> Data;

};

double& TestClass::GetElement()
{
  return this->Data[1];
}

TestClass::TestClass()
{
  this->Data.push_back(0);
  this->Data.push_back(1);
  this->Data.push_back(2);
}

void TestClass::OutputData()
{
  for(unsigned int i = 0; i < this->Data.size(); i++)
    {
    std::cout << Data[i] << " ";
    }
}

int main(int argc, char *argv[])
{
  TestClass test;
  test.OutputData();
  std::cout << std::endl;

  double& element = test.GetElement();

  element = 5;

  test.OutputData();

  return 0;
}
