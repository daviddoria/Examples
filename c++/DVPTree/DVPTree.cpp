#include <iostream>

#include "metric_space_search.hpp"

namespace boost {
  enum vertex_data_t { vertex_data };

  BOOST_INSTALL_PROPERTY(vertex, data);
};

int main(int argc, char *argv[])
{

  typedef boost::adjacency_list<boost::vecS,
                                boost::vecS,
                                boost::undirectedS,
                                boost::property< boost::vertex_data_t, boost::square_topology<>::point_type >
                                > Graph;

  Graph g;

  typedef boost::graph_traits<Graph>::vertex_descriptor VertexType;

    //ReaK::pp::dvp_tree<Key, Topology, PositionMap> tree;
  typedef ReaK::pp::dvp_tree<VertexType, boost::square_topology<>, boost::property_map<Graph, boost::vertex_data_t>::type > TreeType;

  //boost::rectangle_topology<boost::minstd_rand> myTopology;
  boost::square_topology<boost::minstd_rand> myTopology;
  boost::property_map<Graph, boost::vertex_data_t>::type positionMap;
  TreeType tree(g, myTopology, positionMap);

  return 0;
}
