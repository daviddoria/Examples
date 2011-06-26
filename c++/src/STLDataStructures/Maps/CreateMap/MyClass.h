#ifndef MYCLASS_H
#define MYCLASS_H

#include <iostream>
#include <map>
#include <string>

class MyClass
{
	std::map <std::string, int> MyIntMap;
	std::map <std::string, double> MyDoubleMap;
	public:
	
		
	MyClass()
	{
		MyIntMap["one"] = 1;
		MyIntMap["two"] = 2;
		
		MyDoubleMap["one"] = 1.1;
		MyDoubleMap["two"] = 2.2;
	}
	
	/*
	int GetValue(std::string &MyString)
	{
		return MyMap[MyString];
	}
	*/
	
	int GetIntVal(const std::string &MyString)
	{
		//return MyMap[MyString];
		std::map<std::string, int>::iterator iter;
		iter = MyIntMap.find(MyString);
		if(iter == MyIntMap.end())
			return -1; //value not found!
		else
		{
			int Value = iter->second;
			return Value;
		}
	}
	
	double GetDoubleVal(const std::string &MyString)
	{
		//return MyMap[MyString];
		std::map<std::string, double>::iterator iter;
		iter = MyDoubleMap.find(MyString);
		if(iter == MyDoubleMap.end())
			return -1; //value not found!
		else
		{
			double Value = iter->second;
			return Value;
		}
	}
	
	/*
	int GetValue(const char* MyString)
	{
		return MyMap[MyString];
	}
	*/
};

#endif