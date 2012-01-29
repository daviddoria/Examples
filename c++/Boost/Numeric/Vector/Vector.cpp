#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

int main ()
{
  //using namespace boost::numeric::ublas;
  boost::numeric::ublas::vector<double> v(3);

  for (unsigned i = 0; i < v.size(); ++i)
  {
    v(i) = 3 * i;
  }

  std::cout << v << std::endl;

  return 0;
}