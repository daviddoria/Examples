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

class Pilot : protected Person
{
  private:
			  
  public:
    void setName(std::string &name);
    Pilot(std::string &name) : Person(name) {m_Name = name;}
    //Pilot(std::string &name) {m_Name = name;} // does not work!!!
};

class BlimpPilot : protected Pilot
{		
  public:
    void setName(std::string &name);
    BlimpPilot(std::string &name) : Pilot(name) {m_Name = name;}
		
};

int main(int argc, char *argv[])
{
  std::string name = "test pilot";
  
  BlimpPilot Plt(name);
  
  return 0;
}
