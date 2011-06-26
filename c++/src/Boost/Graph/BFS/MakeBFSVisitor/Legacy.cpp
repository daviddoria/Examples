
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/pending/indirect_cmp.hpp>
#include <boost/pending/integer_range.hpp>

#include <iostream>

int main()
{
  typedef boost::adjacency_list < boost::vecS, boost::vecS, boost::undirectedS > graph_t;

  graph_t g;

  boost::add_edge(0, 1, g);
  boost::add_edge(0, 2, g);
  boost::add_edge(1, 3, g);

  unsigned int numberOfVertices = 4;
  // Typedefs
  typedef boost::graph_traits < graph_t >::vertex_descriptor Vertex;
  typedef boost::graph_traits < graph_t >::vertices_size_type Size;
  typedef Size* Iiter;

  // a vector to hold the discover time property for each vertex
  Size time = 0;
  std::vector<Size> discoverTime(boost::num_vertices(g));

  // not the correct return type template parameters
  //boost::bfs_visitor<> visitor = boost::make_bfs_visitor(boost::stamp_times(discoverTime.begin(), time, boost::on_discover_vertex()));
  
  // compile error
  //boost::breadth_first_search(g, 0, boost::visitor(boost::make_bfs_visitor(boost::stamp_times(discoverTime.begin(), time, boost::on_discover_vertex()))));
  
  // compile error
  //boost::breadth_first_search(g, 0, boost::make_bfs_visitor(boost::stamp_times(discoverTime.begin(), time, boost::on_discover_vertex())));
  
  // compile error
  //boost::breadth_first_search(g, 0, boost::stamp_times(discoverTime.begin(), time, boost::on_discover_vertex()));
  
  // compile error
  //boost::breadth_first_search(g, 0, boost::visitor(boost::stamp_times(discoverTime.begin(), time, boost::on_discover_vertex())));
  
  
  
  
  boost::breadth_first_search(g, 0, boost::time_stamper<unsigned int, Size, unsigned int>());
  

  return EXIT_SUCCESS;
}