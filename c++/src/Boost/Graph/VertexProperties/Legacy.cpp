#include <iostream>
#include <string>

#include <boost/graph/undirected_graph.hpp>

typedef boost::property<boost::vertex_name_t, std::string> VertexProperty;
typedef boost::undirected_graph<VertexProperty> Graph;

int main(int,char*[])
{
  // Create a graph object
  Graph g(3);

  // Assign "Vertex0" as the name of the 0th vertex
  g[0] = "Vertex0";

  // Assign "Vertex0" as the name of the 0th vertex
  boost::property_map<Graph, boost::vertex_name_t>::type value = boost::get(boost::vertex_name_t(), g);
  boost::put(value, 0, "Vertex0");

  return 0;
}

===============================
#include <iostream>
#include <string>

#include <boost/graph/undirected_graph.hpp>

typedef boost::property<boost::vertex_name_t, std::string> VertexProperty;
typedef boost::undirected_graph<VertexProperty> Graph;

// vecS is a "Selector type" of std::vector

// Note that these are the same:
// boost::graph_traits<graph_t>::vertex_descriptor vertex_descriptor
// typedef graph_t::vertex_descriptor vertex_descriptor;

int main(int,char*[])
{
  // Create a graph object
  Graph g;

  Graph::vertex_descriptor v0 = g.add_vertex();

  // This does not work
  //g[v0] = "Vertex0";

  // This works
  boost::property_map<Graph, boost::vertex_name_t>::type value = boost::get(boost::vertex_name_t(), g);
  boost::put(value, v0, "Vertex0");

  // This does not work
  //Graph::vertex_descriptor v1 = g.add_vertex("Vertex1");

  // This works
  //VertexProperty v1name;
  //v1name.m_value = "Vertex1";

  // This works
  VertexProperty v1name("Vertex1"); // Note that duplicate names are allowed

  Graph::vertex_descriptor v1 = g.add_vertex(v1name);

  typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
  IndexMap index = get(boost::vertex_index, g);

  typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
  std::pair<vertex_iter, vertex_iter> vertexPair;
  for (vertexPair = boost::vertices(g); vertexPair.first != vertexPair.second; ++vertexPair.first)
    {
    std::cout << index[*vertexPair.first] <<  " : " << value[*vertexPair.first] << std::endl;
    }

  return 0;
}
