#include <iostream>
#include <string>

int main(int argc, char *argv[])
{
  std::string sentence = "hello world test";

  std::string wordToReplace = "world";
  std::string replaceWith = "mars";

  sentence.replace(sentence.find(wordToReplace),
               wordToReplace.size(),
               replaceWith);

  std::cout << sentence;

  return 0;
}
