#include <iostream>

#include "itkGraph.h"
#include "itkBoykovGraphTraits.h"
#include "itkBoykovMinCutGraphFilter.h"

int main( int argc, char * argv[] )
{
	typedef itk::BoykovGraphTraits<short, 3> GraphTraitsType;

	typedef itk::Graph<GraphTraitsType>           GraphType;
	GraphType::Pointer graph = GraphType::New();

	typedef GraphType::NodePointerType      NodePointerType;
	typedef GraphType::EdgePointerType      EdgePointerType;
 
	std::cout << "Num Nodes: " << graph->GetTotalNumberOfNodes() << std::endl;
	std::cout << "Num Edges: " << graph->GetTotalNumberOfEdges() << std::endl;

  	// Create graph nodes
	NodePointerType Nodes[2];
  
	/*
	for( unsigned int i = 0; i < 2; i++ )
	{
		Nodes[i] = graph->CreateNewNode( 2 );
	}
	*/
	
	/*
	for( unsigned int i = 0; i < 2; i++ )
	{
		Nodes[i] = graph->CreateNewNode(0);
	}
	*/
	
	for( unsigned int i = 0; i < 2; i++ )
	{
		Nodes[i] = graph->CreateNewNode();
	}
	
	std::cout << std::endl;
	std::cout << "Num Nodes: " << graph->GetTotalNumberOfNodes() << std::endl;
	std::cout << "Num Edges: " << graph->GetTotalNumberOfEdges() << std::endl;

	
	/*
	Nodes[0]->IsSink = true;
	Nodes[1]->IsSink = false;
	
	Nodes[0]->IsActive = true;
	Nodes[1]->IsActive = true;
	*/
	
	/*
	bool test = true;
	bool test2;
	//get pointers to the newly created nodes
	for( unsigned int i = 0; i < 2; i++ )
	{
		Nodes[i] = graph->GetNodePointer( i );
		std::cout << "Node " << i << " : " << Nodes[i] << std::endl;
	}
*/
     //create three edges between nodes 0 and 1, each with weight 2
	//graph->CreateNewEdge( Nodes[0], Nodes[1], 2 );
	
	graph->CreateNewEdge( Nodes[0], Nodes[1]);
	
	std::cout << std::endl;
	std::cout << "Num Nodes: " << graph->GetTotalNumberOfNodes() << std::endl;
	std::cout << "Num Edges: " << graph->GetTotalNumberOfEdges() << std::endl;


	// Set the reverse edges
	graph->SetAllReverseEdges();

	//perform the cut
	typedef itk::BoykovMinCutGraphFilter  <GraphType> FilterType;
	FilterType::Pointer filter = FilterType::New();
	filter->SetInput(graph);
	filter->Update();

	//see which nodes are sinks
	typedef GraphType::NodeIterator         NodeIteratorType;
	typedef GraphType::NodeIdentifierType   NodeIdentifierType;
	NodeIteratorType nit( graph );
	for( nit.GoToBegin(); !nit.IsAtEnd(); ++nit )
	{
		NodePointerType node = nit.GetPointer();
		NodeIdentifierType Id = graph->GetNodeIdentifier( node );
		std::cout << "Id: " << Id << std::endl;
		
		node = graph->GetNodePointer( Id );
		
		std::cout << "node: " << node << " Nodes[0]: " << Nodes[0] << " Nodes[1]: " << Nodes[1] << std::endl;
		bool IsSink = node->IsSink;
		
		if(IsSink)
		{
			std::cout << "Node Id: " << Id << " is in group 1." << std::endl;
		}
		else
		{
			std::cout << "Node Id: " << Id << " is in group 0." << std::endl;
		}
	}
	
	for(unsigned int i = 0; i < 2; i++)
	{
		if(Nodes[i]->IsSink)
		{
			std::cout << "Nodes[" << i << "] is in group 1." << std::endl;
		}
		else
		{
			std::cout << "Nodes[" << i << "]  is in group 0." << std::endl;
		}
	}
			
	//get the cut weight (min cut = max flow)
	typedef GraphType::NodeWeightType       NodeWeightType;
	NodeWeightType maxflow = filter->GetMaxFlow();
	std::cout << "Max Flow: " << maxflow << std::endl;
	
	return EXIT_SUCCESS;
	
}