#include <iostream>
#include <string>
#include <algorithm>
#include <functional>

void TestEmpty();
void TestConcatenate();
void RemoveCharacters();
void Compare();
void TestFind();

int main(int argc, char *argv[])
{
  //TestEmpty();
  //TestConcatenate();
  //RemoveCharacters();
  //Compare();
  TestFind();

  return 0;
}

void TestFind()
{
  std::string hello = "hello";

  int pos = hello.find("he"); // should return 0
  std::cout << "pos of 'he' " << pos << std::endl;

  pos = hello.find("lo"); // should return 3
  std::cout << "pos of 'lo' " << pos << std::endl;

  pos = hello.find("test"); // should return -1
  std::cout << "pos of 'test' " << pos << std::endl;
}
void Compare()
{
  {
  std::string String1 = "test";
  std::string String2 = "hello";

  if (std::equal(String1.begin(), String1.end(), String2.begin()))
  {
   std::cout << "Match" << std::endl;
  }
  else
  {
   std::cout << "No match." << std::endl;
  }
  }

  {
  std::string String1 = "test";
  std::string String2 = "test";

  if (std::equal(String1.begin(), String1.end(), String2.begin()))
  {
   std::cout << "Match" << std::endl;
  }
  else
  {
   std::cout << "No match." << std::endl;
  }
  }

}

void RemoveCharacters()
{

  std::string MyString = "hello,world123";
  std::cout << "Original: " << MyString << std::endl;

  //Remove all punctuation
  MyString.erase(
  std::remove_if(MyString.begin(), MyString.end(), &ispunct),
  MyString.end());

  std::cout << "Punctuation removed: " << MyString << std::endl;

  //Remove all numbers
  MyString.erase(
  std::remove_if(MyString.begin(), MyString.end(), &isdigit),
  MyString.end());

  std::cout << "Numbers removed: " << MyString << std::endl;

  //Remove non alphanumeric characters
  std::string::iterator rem = std::remove_if(MyString.begin(),MyString.end(),std::not1(std::ptr_fun(&::isalnum)));
  MyString.erase(rem, MyString.end());
}

void TestEmpty()
{
  std::string test;

  if(test.empty())
  {
    std::cout << "empty." << std::endl;
  }

}

void TestConcatenate()
{
  std::string a = "hello";
  std::string b = "world";

  std::cout << a+b << std::endl;
}

