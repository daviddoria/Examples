class MyClass
{
public:

 int const& GetA(){return A;}
 int A;
};
 
int main()
{
 
 MyClass myClass;
 int b = myClass.GetA();
 return 0;
}