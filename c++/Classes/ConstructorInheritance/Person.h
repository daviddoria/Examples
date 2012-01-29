#ifndef PERSON_H
#define PERSON_H

#include <iostream>
#include <string>

class Person
{
  private:

	  
  protected:
    std::string m_Name;
	  
  public:
    Person(std::string name)
    {
      m_Name = name;
    }
		
};


#endif