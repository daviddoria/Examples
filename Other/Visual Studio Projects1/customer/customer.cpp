
#include <string.h>
#include "customer.h"

Customer::Customer() //default constructor
{
	ID=0;
	Name=" ";
}

Customer::Customer(int theID,string theName)
{
	ID=theID;
	Name=theName;
}



int Customer::GetID()const
{return ID;}

	
string Customer::GetName()const
{return Name;}


void Customer::AddJob(string newJob)
{
	Jobs.push_back(newJob);
}

void Customer::RemoveJob(int JobLocation)
{
	Jobs[JobLocation]=" ";
}