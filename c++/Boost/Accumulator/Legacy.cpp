#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/framework/accumulator_set.hpp>
#include <boost/accumulators/statistics/tail_quantile.hpp>
#include <boost/accumulators/statistics/tail.hpp>
#include <boost/accumulators/statistics.hpp>

int main () 
{
  
  typedef accumulator_set<double, stats<boost::accumulators::tag::tail_quantile<left> > > accumulator_t_left;
  typedef accumulator_set<double, stats<boost::accumulators::tag::tail_quantile<right> > > accumulator_t_right;
  
  accumulator_t_right acc0( boost::accumulators::tag::tail<right>::cache_size = c );
  {
  // here push your data in to acc0
  acc0(r);
  }
  
  //get the quantile
  double tqueryP = quantile(acc0, quantile_probability = 0.50 );
	return 0;
}
