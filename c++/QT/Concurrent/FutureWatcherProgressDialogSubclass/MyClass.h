#ifndef MyClass_H
#define MyClass_H

#include <iostream>

class ParentClass
{
public:

  virtual void LongFunction()
  {
    for( int count = 0; count < 5; count++ )
    {
      sleep( 1 );
      std::cout << "Ping long!" << std::endl;
    }
  }
};

class ChildClass : public ParentClass
{
public:

  void LongFunction()
  {
    for( int count = 0; count < 5; count++ )
    {
      sleep( 1 );
      std::cout << "Ping long!" << std::endl;
    }
  }
};

#endif
