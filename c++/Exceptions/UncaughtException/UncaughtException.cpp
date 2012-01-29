#include <exception>
#include <iostream>
#include <string>

class Test
{
public:
   Test( std::string msg ) : m_msg( msg )
   {
      std::cout << "In Test::Test(\"" << m_msg << "\")" << std::endl;
   }
   ~Test( )
   {
      std::cout << "In Test::~Test(\"" << m_msg << "\")" << std::endl
         << "        std::uncaught_exception( ) = "
         << std::uncaught_exception( )
         << std::endl;
   }
private:
    std::string m_msg;
};

// uncaught_exception will be true in the destructor
// for the object created inside the try block because
// the destructor is being called as part of the unwind.

// int main( void )
// {
//   Test test( "inside try block" ); // this doesn't work. If there is not a try/catch block, the exception does not unwind and call the destructors
//   throw 1;
//   return 0;
// }


int main( void )
{
  Test t1( "outside try block" );
  try
  {
      Test t2( "inside try block" );
      throw 1;
  }
  catch (...)
  {
    // Since the exception is caught, the program does not terminate here, and therefore also calls t1's destructor.
  }
}