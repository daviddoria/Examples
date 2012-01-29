#include <iostream>

class Test
{
  private:
	  int A,B,C;
  public:
    Test& setB(const int b)
    {
	    B = b;
	    return *this;
    }
    
    Test& setC(const int c)
    {
	    C = c;
	    return *this;
    }
    
    Test(const int a) : A(a) 
    {
	    B = 0;
	    C = 0;
    }
    
    void Output()
    {
	    std::cout << A << " " << B << " " << C << std::endl;
	    
    }
};

int main(int argc, char *argv[])
{
  Test T(1).setB(4);
  T.Output();
  
  return 0;
}
