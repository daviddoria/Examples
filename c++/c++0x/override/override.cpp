class Base 
{
  virtual void Foo() {};
}; 

// Not supported until gcc 4.7
class Derived : public Base 
{ 
  public: virtual void Foo() override {};
};

int main()
{
  
  return 0;
}
