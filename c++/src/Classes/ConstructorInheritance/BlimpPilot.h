#ifndef BLIMPPILOT_H
#define BLIMPPILOT_H

#include <iostream>

#include "Pilot.h"

class BlimpPilot : protected Pilot
{
	
				
	public:
		void setName(std::string &name);
		BlimpPilot(std::string &name) : Pilot(name) {m_Name = name;}
		
};

#endif