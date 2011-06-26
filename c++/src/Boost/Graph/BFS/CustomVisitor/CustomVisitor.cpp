#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>

#include <iostream>

class custom_bfs_visitor : public boost::default_bfs_visitor
{
public:

  template < typename Vertex, typename Graph >
  void discover_vertex(Vertex u, const Graph & g) const
  {
    std::cout << u << std::endl;
  }
};

int main()
{
  typedef boost::adjacency_list < boost::vecS, boost::vecS, boost::undirectedS > graph_t;

  graph_t g;

  boost::add_edge(0, 1, g);
  boost::add_edge(0, 2, g);
  boost::add_edge(1, 3, g);
  boost::add_edge(0, 4, g);
  
  /*
       v0
      / | \
     /  |  \
    v1 v2   v4
   /
  /
 v3
  */

  custom_bfs_visitor visitor;
  boost::breadth_first_search(g, boost::vertex(0, g), boost::visitor(visitor));

  // Output: 0 1 2 4 3
  return EXIT_SUCCESS;
}