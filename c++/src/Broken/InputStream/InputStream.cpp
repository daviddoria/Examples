#include <iostream>
#include <string>
#include <vector>
#include <sstream>

using namespace std;

int main(int argc, char *argv[])
{
    
  double x,y,z;	
  
  std::string numbers_str = "1,2,3";
  std::vector < std::string > numbers; //we'll put all of the tokens in here 
  std::string  temp;

  while (numbers_str.find(",", 0) != std::string::npos) // "as many as possible"
  {
    size_t  pos = numbers_str.find(",", 0); //store the position of the delimiter
    temp = numbers_str.substr(0, pos);      //get the token
    numbers_str.erase(0, pos + 1);          //erase it from the source 
    numbers.push_back(temp);                //and put it into the array
  }

  numbers.push_back(numbers_str);           //the last token is all alone 
  
  for(unsigned int i = 0; i < numbers.size(); i++)
  {
    std::cout << numbers[i] << " ";
  }
  
  return 0;
}
