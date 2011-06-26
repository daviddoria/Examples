#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/pending/indirect_cmp.hpp>
#include <boost/pending/integer_range.hpp>

#include <iostream>


template < typename TimeMap > class bfs_time_visitor : public boost::default_bfs_visitor
{
  typedef typename boost::property_traits < TimeMap >::value_type T;
public:
  bfs_time_visitor(TimeMap tmap, T & t):m_timemap(tmap), m_time(t) { }
  template < typename Vertex, typename Graph >
  void discover_vertex(Vertex u, const Graph & g) const
  {
    put(m_timemap, u, m_time++);
    std::cout << u << std::endl;
  }
  TimeMap m_timemap;
  T & m_time;
};


int main()
{
  using namespace boost;
  // Select the graph type we wish to use
  typedef adjacency_list < vecS, vecS, undirectedS > graph_t;

  graph_t g;

  add_edge(0, 1, g);
  add_edge(0, 2, g);
  add_edge(1, 3, g);
  add_edge(0, 4, g);

  unsigned int numberOfVertices = 4;
  // Typedefs
  typedef graph_traits < graph_t >::vertex_descriptor Vertex;
  typedef graph_traits < graph_t >::vertices_size_type Size;
  typedef Size* Iiter;

  // a vector to hold the discover time property for each vertex
  std::vector < Size > dtime(num_vertices(g));

  Size time = 0;
  bfs_time_visitor < Size * >vis(&dtime[0], time);
  breadth_first_search(g, vertex(0, g), visitor(vis));

  return EXIT_SUCCESS;
}