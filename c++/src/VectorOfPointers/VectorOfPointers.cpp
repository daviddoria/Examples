#include <iostream>
#include <vector>

class TestClass
{
public:
  TestClass();

  void OutputData();

  //std::vector<double&> GetOddElements(); // can't do this!
  std::vector<double*> GetOddElements();

  double& GetElement();

private:
  std::vector<double> Data;

};

std::vector<double*> TestClass::GetOddElements()
{
  std::vector<double*> oddElements;

  // Add pointers to the values at odd ids
  oddElements.push_back(&(this->Data[1]));
  oddElements.push_back(&(this->Data[3]));

  return oddElements;
}

TestClass::TestClass()
{
  this->Data.push_back(0);
  this->Data.push_back(1);
  this->Data.push_back(2);
  this->Data.push_back(3);
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

  std::vector<double*> oddElements = test.GetOddElements();
  *(oddElements[0]) = 6;

  std::cout << std::endl;

  test.OutputData();

  return 0;
}
