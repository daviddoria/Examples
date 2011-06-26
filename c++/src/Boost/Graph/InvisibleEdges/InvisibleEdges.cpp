#include <iostream>
#include <string>
#include <fstream>

#include <boost/graph/undirected_graph.hpp>
#include <boost/graph/graphviz.hpp>

struct StyleProperty
{
  std::string style;
};

typedef boost::undirected_graph<boost::no_property, StyleProperty> Graph;

// Normal edges are written to a graphviz dot file with:
// 1--2
// To make an edge invisible, it must be written as:
// 1--2 [style=invis];

int main(int argc, char*argv[])
{
  // Verify arguments
  if(argc < 2)
  {
    std::cerr << "Required: filename.dot" << std::endl;
    return -1;
  }

  // Parse arguments
  std::string filename = argv[1];
  std::ofstream fout(filename.c_str());

  // Create a graph
  Graph g;

  // Add 3 vertices
  Graph::vertex_descriptor v0 = g.add_vertex();
  Graph::vertex_descriptor v1 = g.add_vertex();
  Graph::vertex_descriptor v2 = g.add_vertex();

  // Setup "styles" for the edges
  StyleProperty styleInvisible;
  styleInvisible.style = "invis";

  StyleProperty styleNormal;
  styleNormal.style = "normal";

  // Add two edges. The first will be invisible.
  boost::add_edge(v0,v1,styleInvisible, g);
  boost::add_edge(v1,v2,styleNormal, g);

  // Setup the dynamic property which will be used to write the graph with edge properties
  boost::dynamic_properties dp;
  dp.property("style", get(&StyleProperty::style, g));
  dp.property("node_id", get(boost::vertex_index, g));

  // Write out the graph
  boost::write_graphviz_dp(std::cout, g, dp);
  boost::write_graphviz_dp(fout, g, dp);
  
  return 0;
}
