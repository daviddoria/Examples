#include <iostream>
#include <string>

using namespace std;
		
int main(int argc, char *argv[])
{
  string Hello = "Hello";
  string Goodbye = "Goodbye";

  cout << Hello.compare(Goodbye) << " (should be != 0)" << endl;
  cout << Hello.compare(Hello) << " (should be == 0)" << endl;
  cout << Hello.compare("Hello") << " (should be == 0)" << endl;

  return 0;
}
