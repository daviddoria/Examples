#include <iostream>
#include <string>
#include <sstream>

void StringToNumber();
void TestClear();
void NumberToString();
void Test();

int main(int argc, char *argv[])
{
  TestClear();
  //NumberToString();
  //Test();


  return 0;
}

void Test()
{
  /*
  StringToNumber();
  std::stringstream TestStream1;
  TestStream1 << "hello" << 1 << std::endl;
  std::cout << TestStream1.str();

  //TestStream1.clear();
  //TestStream1.str().erase();
  TestStream1.str("");
  TestStream1 << "hello" << 2 << std::endl;
  std::cout << TestStream1.str();
  */
  std::stringstream TestStream2;
  TestStream2 << "hello";
  std::cout << TestStream2.str();
  std::cout << std::endl;

  TestStream2 << "world";
  std::cout << TestStream2.str();

}

void TestClear()
{
  std::stringstream TestStream1;
  TestStream1 << "hello" << 1 << std::endl;
  std::cout << TestStream1.str();

  TestStream1.str(""); // works
  TestStream1 << "hello" << 2 << std::endl;
  std::cout << TestStream1.str();

  TestStream1.str("hello3");
  std::cout << TestStream1.str();

  TestStream1.clear(); // does NOT work
  TestStream1 << "goodbye";
  std::cout << TestStream1.str();

  ///
  float temp;
  
  std::cout << std::endl;
  TestStream1.str("");
  TestStream1 << "7";
  std::cout << TestStream1.str() << std::endl;
  TestStream1 >> temp;
  std::cout << temp << std::endl;

  TestStream1.str("");
  TestStream1 << "8";
  std::cout << TestStream1.str() << std::endl;
  TestStream1 >> temp;
  std::cout << temp << std::endl;
}

void StringToNumber()
{
  std::string strNumber = "23.4";
  std::cout << "strNumber = " << strNumber << std::endl;
  std::stringstream ss;
  ss << strNumber;
  double dNumber;
  ss >> dNumber;
  std::cout << "dNumber = " << dNumber << std::endl;
}

void NumberToString()
{
  int intNumber = 13;

  //put the number in a stringstream
  std::stringstream ss;
  ss << intNumber;

  //get the number out of the stringstream as a string
  std::string strNumber;
  ss >> strNumber;

  std::cout << "strNumber = " << strNumber << std::endl;
}
