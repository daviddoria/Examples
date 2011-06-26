#include <iostream>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>

//#include <boost/pending/indirect_cmp.hpp>
//#include <boost/pending/integer_range.hpp>

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS> MyGraphType;

int main(int,char*[])
{
  MyGraphType g;
  boost::add_edge(0, 1, g);
  boost::add_edge(0, 2, g);
  boost::add_edge(1, 3, g);

  int time = 0;
  std::vector<double> dtime(boost::num_vertices(g));
  std::vector<double> ftime(boost::num_vertices(g));

  boost::breadth_first_search(g,
    boost::make_bfs_visitor(
     std::make_pair(boost::stamp_times(dtime.begin(), time, boost::on_discover_vertex()),
                    boost::stamp_times(ftime.begin(), time, boost::on_finish_vertex())))
  );

  /*
  boost::breadth_first_search(g, *boost::vertices(g).first,
    visitor(boost::make_bfs_visitor(
     std::make_pair(boost::stamp_times(dtime.begin(), time, boost::on_discover_vertex()),
                    boost::stamp_times(ftime.begin(), time, boost::on_finish_vertex()))))
  );
*/

/*
  boost::breadth_first_search(g, 0,
   boost::make_bfs_visitor(
    std::make_pair(boost::record_distances(dtime, boost::on_tree_edge()),
    std::make_pair(boost::record_predecessors(p.begin(),
                                              boost::on_tree_edge()),
                   copy_graph(G_copy, boost::on_examine_edge())))) );
*/
  return 0;
}


