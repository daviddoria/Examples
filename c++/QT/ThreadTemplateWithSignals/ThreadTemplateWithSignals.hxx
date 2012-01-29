template <typename T>
void ThreadTemplateWithSignals<T>::run()
{
  for( int count = 0; count < 5; count++ )
    {
    std::cout << count << std::endl;;
    }
}
