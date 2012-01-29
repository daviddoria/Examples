#include <iostream>

class A 
{ 
	virtual void MyFunc() const = 0; 
}; 

class B : public A 
{ 
	virtual void MyFunc() const = 0; 
}; 

class C : public B 
{ 
	private:
		mutable int MyVar;
		
	public:
	void MyFunc() const
	{
		std::cout << "hello world";
		MyVar = 2;
	}
}; 

int main()
{
	C MyC;
	MyC.MyFunc();
	return 0;
}