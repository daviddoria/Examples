#include <iostream>
#include <vector>

class TestClass
{
public:
  TestClass();

  void OutputData();

  std::vector<double>& GetData();

private:
  std::vector<double> Data;

};

std::vector<double>& TestClass::GetData()
{
  return this->Data;
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
  std::cout << std::endl;
}

int main(int argc, char *argv[])
{
  TestClass test;
  test.OutputData();

  std::vector<double>& data = test.GetData();
  //std::vector<double> data = test.GetData();
  data[0] = 5;

  //test.GetData()[0] = 5;

  test.OutputData();

  return 0;
}
