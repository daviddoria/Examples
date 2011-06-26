

#include <vector>

using namespace std;

class Customer
{
public: // functions
	Customer();
	Customer (int theID, string theName);

	string GetID()const;
	string GetName()const;
	void AddJob(string newJob);
	void RemoveJob(int JobLocation);
	
private: //variables
	int ID;
	string Name;
	vector <string> Jobs;
}