#include <iostream>

#include "itkGraph.h"
#include "itkBoykovGraphTraits.h"
#include "itkBoykovMinCutGraphFilter.h"

int main( int argc, char * argv[] )
{
	//setup types
	typedef itk::BoykovGraphTraits<short, 3> GraphTraitsType;
	typedef itk::Graph<GraphTraitsType>           GraphType;
	typedef GraphType::NodePointerType      NodePointerType;
	typedef GraphType::EdgePointerType      EdgePointerType;
	
	//create a new graph
	GraphType::Pointer graph = GraphType::New();

  	// Create graph nodes
	NodePointerType Nodes[3];
  	
	for( unsigned int i = 0; i < 2; i++ )
	{
		Nodes[i] = graph->CreateNewNode();
	}
	
	/*
	//these lines don't change the behavior
	Nodes[0]->IsSink = true; //set node 0 to be a sink
	Nodes[1]->IsSink = false; //set node 1 to be a source
	*/
	 
	//create an edge between nodes 0 and 1 with weight 2
	graph->CreateNewEdge( Nodes[0], Nodes[1], 2);
	
	//verify that the graph was created correctly
	std::cout << std::endl;
	std::cout << "Num Nodes: " << graph->GetTotalNumberOfNodes() << std::endl;
	std::cout << "Num Edges: " << graph->GetTotalNumberOfEdges() << std::endl;

	// Set the reverse edges (doesn't change the behavior)
	//graph->SetAllReverseEdges();

	//perform the cut
	typedef itk::BoykovMinCutGraphFilter  <GraphType> FilterType;
	FilterType::Pointer CutFilter = FilterType::New();
	CutFilter->SetInput(graph);
	CutFilter->Update();
	
	//see which nodes are sinks
	typedef GraphType::NodeIterator         NodeIteratorType;
	typedef GraphType::NodeIdentifierType   NodeIdentifierType;
	NodeIteratorType nit( graph );
	for( nit.GoToBegin(); !nit.IsAtEnd(); ++nit )
	{
		NodePointerType node = nit.GetPointer();
		NodeIdentifierType Id = graph->GetNodeIdentifier( node );
		
		node = graph->GetNodePointer( Id );
		
		if(node->IsSink)
		{
			std::cout << "Node Id: " << Id << " is a sink." << std::endl;
		}
		else
		{
			std::cout << "Node Id: " << Id << " is a source." << std::endl;
		}
	}
	
			
	//get the cut weight (min cut = max flow)
	typedef GraphType::NodeWeightType       NodeWeightType;
	NodeWeightType maxflow = CutFilter->GetMaxFlow();
	std::cout << "Max Flow: " << maxflow << std::endl;  //expected: 2, actual: 0
	
	return EXIT_SUCCESS;
	
}