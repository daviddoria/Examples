#include <iostream>
#include <boost/graph/adjacency_list.hpp>

// Create a struct to hold properties for each vertex
struct VertexProperties
{
  int VertexIntProperty;
  std::string VertexStringProperty;
};

// Create a struct to hold properties for each edge
struct EdgeProperties
{
  int EdgeIntProperty;
  std::string EdgeStringProperty;
};

// Define the type of the graph
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperties, EdgeProperties> Graph;

int main(int,char*[])
{
  // Create a graph object
  Graph g;

  // Add two vertices
  Graph::vertex_descriptor v0 = boost::add_vertex(g);
  Graph::vertex_descriptor v1 = boost::add_vertex(g);

  // Add an edge
  std::pair<Graph::edge_descriptor, bool> e01 = boost::add_edge(v0, v1, g);

  // Set and output the properties of a single vertex
  g[v0].VertexIntProperty = 5;
  g[v0].VertexStringProperty = "mystring";
  std::cout << g[v0].VertexIntProperty <<  " : " << g[v0].VertexStringProperty <<  std::endl;
  
  // Set and output the properties of a single edge
  g[e01.first].EdgeIntProperty = 5;
  g[e01.first].EdgeStringProperty = "mystring";
  std::cout << g[e01.first].EdgeIntProperty <<  " : " << g[e01.first].EdgeStringProperty <<  std::endl;
  
  // Set and output the properties of each vertex
  std::pair<Graph::vertex_iterator, Graph::vertex_iterator> vertexIteratorRange = boost::vertices(g);
  for(Graph::vertex_iterator vertexIterator = vertexIteratorRange.first; vertexIterator != vertexIteratorRange.second; ++vertexIterator)
    {
    g[*vertexIterator].VertexIntProperty = 5;
    g[*vertexIterator].VertexStringProperty = "test";
    std::cout << g[*vertexIterator].VertexIntProperty <<  " : " << g[*vertexIterator].VertexStringProperty <<  std::endl;
    }

  // Set and output the properties of each edge
  std::pair<Graph::edge_iterator, Graph::edge_iterator> edgeIteratorRange = boost::edges(g);
  for(Graph::edge_iterator edgeIterator = edgeIteratorRange.first; edgeIterator != edgeIteratorRange.second; ++edgeIterator)
    {
    g[*edgeIterator].EdgeIntProperty = 5;
    g[*edgeIterator].EdgeStringProperty = "test";
    std::cout << g[*edgeIterator].EdgeIntProperty <<  " : " << g[*edgeIterator].EdgeStringProperty <<  std::endl;
    }

  return 0;
}
