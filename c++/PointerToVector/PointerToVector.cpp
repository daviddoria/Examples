#include <iostream>
#include <vector>

class TestClass
{
public:

  std::vector<double>* GetData();
  void AddData(double);

private:
  std::vector<double> Data;

};

std::vector<double>* TestClass::GetData()
{
  return &(this->Data);
}

void TestClass::AddData(double value)
{
  this->Data.push_back(value);
}

int main(int argc, char *argv[])
{
  TestClass test;
  test.AddData(1.0);
  test.AddData(2.0);

  std::vector<double>* data = test.GetData();

  for(unsigned int i = 0; i < data->size(); i++)
    {
    std::cout << (*data)[i] << " ";
    // std::cout << data[i] << " "; // Does not work
    // std::cout << *(data)[i] << " "; // Does not work
    }
  std::cout << std::endl;

  return 0;
}
