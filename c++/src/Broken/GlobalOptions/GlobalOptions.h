#include <iostream>
#include <string>

using namespace std;

class GlobalOptions 
{
 public:

  string parallel;

  static GlobalOptions& Get()
    {
      static GlobalOptions options;
      return options;
    }

 private:
 
 GlobalOptions() {}
 
 GlobalOptions(const GlobalOptions&) {}
 
 GlobalOptions operator=(const GlobalOptions&) {}
 
};



 
