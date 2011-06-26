#include <iostream>
#include <string>

#include "Person.h"
#include "Pilot.h"

using namespace std;
		
int main(int argc, char *argv[])
{
	
  Pilot Plt;
  std::string name = "test pilot";
  Plt.setName(name);
  return 0;
}