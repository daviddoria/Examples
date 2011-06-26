#include <iostream>
		
void Function(const char* word);

int main(int argc, char *argv[])
{
  Function("hello");
  
  return 0;
}

void Function(const char* word)
{
  std::cout << word << std::endl;
}
