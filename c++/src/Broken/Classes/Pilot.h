#ifndef PILOT_H
#define PILOT_H

#include <iostream>

#include "Person.h"

//class Pilot : public Person
class Pilot : protected Person
{
	private:
				
	public:
		void setName(std::string &name);
};

#endif