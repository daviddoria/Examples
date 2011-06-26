
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/pending/indirect_cmp.hpp>
#include <boost/pending/integer_range.hpp>
 
#include <iostream>
 
int main()
{
  typedef boost::adjacency_list < boost::vecS, boost::vecS, boost::undirectedS > graph_t;
 
  // Create this graph:
    
  /*
       v0
      / | \
     /  |  \
    v1 v2   v4
   /
  /
 v3
  */
  graph_t g;
 
  boost::add_edge(0, 1, g);
  boost::add_edge(0, 2, g);
  boost::add_edge(1, 3, g);
  boost::add_edge(0, 4, g);
  
  typedef boost::graph_traits < graph_t >::vertices_size_type Size;
  
  // a vector to hold the discover time property for each vertex
  unsigned int time = 0;
  Size discoverTime[ boost::num_vertices(g) ];

  boost::breadth_first_search(g, 0u, boost::visitor( boost::make_bfs_visitor(  
  boost::stamp_times(discoverTime, time, boost::on_discover_vertex() ))));
  
  for(unsigned int m=0;m<boost::num_vertices(g);++m)
  {
    std::cout << "v=" << m << " dTime=" << discoverTime[m] << std::endl;;
  }
 
  return EXIT_SUCCESS;
}