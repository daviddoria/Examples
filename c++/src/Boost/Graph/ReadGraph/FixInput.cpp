// http://www.boost.org/doc/libs/1_37_0/libs/graph/doc/read_graphviz.html
#include <iostream>
#include <string>
#include <fstream>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

typedef boost::adjacency_list < boost::vecS, boost::vecS, boost::undirectedS> Graph;

void OutputEdges(const Graph& g);

Graph CreateGraph();
Graph ReadGraph();

int main(int argc, char*argv[])
{
  std::cout << "Created" << std::endl;
  
  OutputEdges(CreateGraph());
  
  std::cout << std::endl << "Read" << std::endl;
  
  OutputEdges(ReadGraph());
  
  return 0;
}

void OutputEdges(const Graph& g)
{
  std::pair<Graph::edge_iterator, Graph::edge_iterator> edgeIteratorRange = boost::edges(g);
  for(Graph::edge_iterator edgeIterator = edgeIteratorRange.first; edgeIterator != edgeIteratorRange.second; ++edgeIterator)
    {
      std::cout << "Edge exists between " << boost::target(*edgeIterator, g) << " and " 
                <<  boost::source(*edgeIterator, g) << std::endl;
    }  
}

Graph CreateGraph()
{
  Graph g(19);

  boost::add_edge(0,1,g);
  boost::add_edge(1,2,g);
  boost::add_edge(2,3,g);
  boost::add_edge(3,4,g);
  boost::add_edge(4,5,g);
  boost::add_edge(5,6,g);
  boost::add_edge(6,7,g);
  boost::add_edge(7,8,g);
  boost::add_edge(7,9,g);
  boost::add_edge(9,10,g);
  boost::add_edge(7,11,g);
  boost::add_edge(11,12,g);
  boost::add_edge(12,13,g);
  boost::add_edge(13,14,g);
  boost::add_edge(13,15,g);
  boost::add_edge(15,16,g);
  boost::add_edge(16,17,g);
  boost::add_edge(17,18,g);
  
  return g;
}


Graph ReadGraph()
{
  std::stringstream ss("graph G { \
0; \
1; \
2; \
3; \
4; \
5; \
6; \
7; \
8; \
9; \
10; \
11; \
12; \
13; \
14; \
15; \
16; \
17; \
18; \
0--1 ; \
1--2 ; \
2--3 ; \
3--4 ; \
4--5 ; \
5--6 ; \
6--7 ; \
7--8 ; \
7--9 ; \
9--10 ; \
7--11 ; \
11--12 ; \
12--13 ; \
13--14 ; \
13--15 ; \
15--16 ; \
16--17 ; \
17--18 ; \
}");

  // Create a graph type with a vertex property to store the id of the vertices in the graphviz file
  typedef boost::property < boost::vertex_name_t, std::string> VertexProperty;
  typedef boost::adjacency_list < boost::vecS, boost::vecS, boost::undirectedS, VertexProperty> GraphFromFile;
  
  // Read the graphviz file
  GraphFromFile graphFromFile;
  boost::dynamic_properties dp;
  
  boost::property_map<GraphFromFile, boost::vertex_name_t>::type name =
    get(boost::vertex_name, graphFromFile);
  dp.property("node_id",name);
  
  boost::read_graphviz(ss,graphFromFile,dp);

  // Create a property_map of the input vertex ids
  boost::property_map<GraphFromFile, boost::vertex_name_t>::type value = boost::get(boost::vertex_name_t(), graphFromFile);
  
  // Create a new graph of the desired type
  Graph graph(boost::num_vertices(graphFromFile));
  
  // Iterate over the edges of the input graph and create edges in the output graph on the corresponding vertices
  std::pair<GraphFromFile::edge_iterator, GraphFromFile::edge_iterator> edgePair;
  for(edgePair = boost::edges(graphFromFile); edgePair.first != edgePair.second; ++edgePair.first)
  {
    std::stringstream ssSource(value[boost::source(*edgePair.first, graphFromFile)]);
    std::stringstream ssTarget(value[boost::target(*edgePair.first, graphFromFile)]);
    unsigned int source = 0;
    unsigned int target = 0;
    ssSource >> source;
    ssTarget >> target;
    boost::add_edge(source, target, graph);
  }
  return graph;
}
