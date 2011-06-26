// Note tested under boost 1.42
#include <vector>
#include <algorithm>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/tail_quantile.hpp>

using namespace boost::accumulators;
//typedef accumulator_set<double, stats<boost::accumulators::tag::tail_quantile<left> > > accumulator_t_left;
typedef accumulator_set<double, stats<boost::accumulators::tag::tail_quantile<right> > > accumulator_t_right;

template <typename T>
class data_filler 
{  
public:
  data_filler(){}
  T operator()() { return rand()/(T)RAND_MAX; }
};

int main(int argc, char** argv)
{
  
  //create some random data
  std::vector<double> data(100);
  std::generate(data.begin(), data.end(), data_filler<double>());
  int c = data.size();//cache size for histogramm.
  
  //create a accumulator.
  accumulator_t_right acc0( boost::accumulators::tag::tail<right>::cache_size = c );
  //fill accumulator 
  for (int j = 0; j < c; ++j)
  {
      acc0(data[j]);
  }
  //ask some questions...
  double tqueryP = quantile(acc0, quantile_probability = 0.50 );
  std::cout << tqueryP << std::endl;

  return 0;
}
////////////////// END snip