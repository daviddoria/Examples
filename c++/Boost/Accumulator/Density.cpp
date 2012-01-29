#include <vector>
#include <algorithm>
#include <iostream>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include <boost/accumulators/statistics/stats.hpp>

using namespace boost;
using namespace boost::accumulators;

typedef accumulator_set<double, features<tag::density> > acc;
typedef iterator_range<std::vector<std::pair<double, double> >::iterator > histogram_type; 
  
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
  
  //create an accumulator
  acc myAccumulator( tag::density::num_bins = 20, tag::density::cache_size = 10);

  //fill accumulator 
  for (int j = 0; j < c; ++j)
  {
    myAccumulator(data[j]);
  }

  histogram_type hist = density(myAccumulator);
  
  double total = 0.0;
  
  for( int i = 0; i < hist.size(); i++ ) 
  {
    std::cout << "Bin lower bound: " << hist[i].first << ", Value: " << hist[i].second << std::endl; 
    total += hist[i].second;
  }
  
  std::cout << "Total: " << total << std::endl; //should be 1 (and it is)
  
  return 0;
}
