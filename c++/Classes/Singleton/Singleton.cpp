#include <iostream>

class Log
{
  public:
    static Log* Instance()
    {
      if (!instance)   // Only allow one instance of class to be generated.
        {
        instance = new Log;
        }

      return instance;

    }
    
    double val;
  
  private:
    Log(){}
    Log& operator=(const Log&); //prevent assignment
    Log(const Log&);//prevent copy construction

    static Log* instance; //store the single instance

};

Log* Log::instance = NULL; 

int main(int argc, char* argv[])
{
  Log::Instance()->val = 2.0;

  std::cout << "val = " << Log::Instance()->val << std::endl;;

  return 0;
}
