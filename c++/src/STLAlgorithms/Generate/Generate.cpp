#include <iostream>
#include <vector>
#include <algorithm>

template <typename T>
class data_filler 
{  
public:
  data_filler(){}
  T operator()() { return rand()/(T)RAND_MAX; }
};

int main (int argc, char *argv[]) 
{
  //create some random data
  std::vector<double> data(10);
  std::generate(data.begin(), data.end(), data_filler<double>());

  for(unsigned int i = 0; i < data.size(); i++)
  {
    std::cout << data[i] << std::endl;
  }
  
  return 0;
}
