#include "MyClass.h"

void LongFunction()
{
  for( int count = 0; count < 5; count++ )
  {
    sleep( 1 );
    std::cout << "Ping long!" << std::endl;
  }
}