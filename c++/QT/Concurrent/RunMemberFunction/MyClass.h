#ifndef MyClass_H
#define MyClass_H

#include <iostream>

class MyClass
{

public:

  void start()
  {
    for( int count = 0; count < 5; count++ )
    {
      sleep( 1 );
      std::cout << "Ping!" << std::endl;
    }
  }

};

#endif
