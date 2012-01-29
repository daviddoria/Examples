template <typename T>
void Output(std::vector<T> &V)
{
  std::cout << std::endl << "other" << std::endl << "-------" << std::endl;
  for(unsigned int i = 0; i < V.size(); i++)
  {
    std::cout << V[i] << std::endl;
  }

}