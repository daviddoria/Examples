/*
 * In the included example, the max centrality of the graph is 14. So specifying
 * the required input argument 14 gives the "first" decomposition of the graph.
 * That is, removal of one edge (with the maximal edge centrality). Specifying a value
 * greater than 14 yields no decomposition.
 *
 * There is no automatic way to set this threshold.
 */

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

// For clustering
#include <boost/graph/bc_clustering.hpp>
#include <boost/graph/iteration_macros.hpp>


// Graph edge properties (bundled properties)
struct EdgeProperties
{
  int weight;
};

typedef boost::adjacency_list< boost::setS, boost::vecS, boost::undirectedS, boost::no_property, EdgeProperties > Graph;
typedef Graph::vertex_descriptor Vertex;
typedef Graph::edge_descriptor Edge;

void WriteGraph(const Graph& g, const std::string& filename);

int main(int argc, char** argv)
{
  // Verify arguments
  if( argc != 2 )
  {
    std::cerr << "USAGE: " << argv[0] << " <Max centrality>" << std::endl;
    return -1;
  }

  // Convert the input argument to a double
  std::stringstream ss;
  ss << argv[1];
  double max_centrality;
  ss >> max_centrality;

  // Create a star graph
  Graph g;

  // Central vertex
  Vertex centerVertex = boost::add_vertex(g);

  // Surrounding vertices
  Vertex v;
  v = boost::add_vertex(g);
  boost::add_edge(centerVertex, v, g);
  v = boost::add_vertex(g);
  boost::add_edge(centerVertex, v, g);
  v = boost::add_vertex(g);
  boost::add_edge(centerVertex, v, g);
  v = boost::add_vertex(g);
  boost::add_edge(centerVertex, v, g);
  v = boost::add_vertex(g);
  boost::add_edge(centerVertex, v, g);
  v = boost::add_vertex(g);
  boost::add_edge(centerVertex, v, g);
  v = boost::add_vertex(g);
  boost::add_edge(centerVertex, v, g);

  // Attach an additional vertex to one of the star arm vertices
  Vertex x = boost::add_vertex(g);
  boost::add_edge(v, x, g);

  // std::map used for convenient initialization
  typedef std::map<Edge, int> StdEdgeIndexMap;
  StdEdgeIndexMap my_e_index;
  // associative property map needed for iterator property map-wrapper
  typedef boost::associative_property_map< StdEdgeIndexMap > EdgeIndexMap;
  EdgeIndexMap e_index(my_e_index);

  // We use setS as edge-container -> no automatic indices
  // -> Create and set it explicitly
  int i = 0;
  BGL_FORALL_EDGES(edge, g, Graph)
  {
    my_e_index.insert(std::pair< Edge, int >( edge, i));
    ++i;
  }

  // Define EdgeCentralityMap
  std::vector< double > e_centrality_vec(boost::num_edges(g), 0.0);
  // Create the external property map
  boost::iterator_property_map< std::vector< double >::iterator, EdgeIndexMap >
          e_centrality_map(e_centrality_vec.begin(), e_index);

  // Define VertexCentralityMap
  typedef boost::property_map< Graph, boost::vertex_index_t>::type VertexIndexMap;
  VertexIndexMap v_index = get(boost::vertex_index, g);
  std::vector< double > v_centrality_vec(boost::num_vertices(g), 0.0);
  // Create the external property map
  boost::iterator_property_map< std::vector< double >::iterator, VertexIndexMap >
          v_centrality_map(v_centrality_vec.begin(), v_index);


  std::cout << "Before" << std::endl;
  print_graph(g);

  BGL_FORALL_EDGES(edge, g, Graph)
  {
    std::cout << edge << ": " << e_centrality_map[edge] << std::endl;
  }

  // Write to graphviz -> illustrate the graph via 'neato -Tps before.dot > before.ps'
  WriteGraph(g, "before.dot");

  // Calculate the vertex and edge centralites
  // Can be used to get an initial impression about the edge centrality values for the graph
  //brandes_betweenness_centrality( g, v_centrality_map, e_centrality_map );

  // Define the done-object:
  // 'false' means here that no normalization is performed, so edge centrality-values can become big
  // If set to 'true', values will range between 0 and 1 but will be more difficult to use for this
  // illustrative example.
  boost::bc_clustering_threshold< double > terminate(max_centrality, g, false);

  //
  // Do the clustering
  // Does also calculate the brandes_betweenness_centrality and stores it in e_centrality_map
  //
  betweenness_centrality_clustering( g, terminate, e_centrality_map );

  // Print the results
  std::cout << "\nAfter" << std::endl;
  print_graph(g);

  BGL_FORALL_EDGES(edge, g, Graph)
  {
    std::cout << edge << ": " <<e_centrality_map[edge] << std::endl;
  }

  // Write to graphviz -> illustrate the graph via 'neato -Tps after.dot > after.ps'
  WriteGraph(g, "after.dot");

  return 0;
}

void WriteGraph(const Graph& g, const std::string& filename)
{
  std::ofstream graphStream;
  graphStream.open(filename.c_str());
  boost::write_graphviz(graphStream, g );
  graphStream.close();
}