
//namespace TestNamespace
//{
  
template<class T, class U, int I> 
void TestNamespace::X<T,U,I>::f()
{
  std::cout << "Primary template" << std::endl;
}

template<class T, int I>
void TestNamespace::X<T, T*, I>::f()
{
  std::cout << "Partial specialization 1" << std::endl;
}

//}