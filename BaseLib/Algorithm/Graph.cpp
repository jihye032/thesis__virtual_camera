

#include "BaseLib/Algorithm/Graph.h"
#include <algorithm>
#include <list>
#include <stack>
#include <algorithm>
#include <queue>
#include <assert.h>
#include <limits>
#include <iostream>

namespace mg
{

Graph::Graph()
{
}

Graph::~Graph()
{
}

GraphNode::GraphNode()
{
	data_ = 0;
	visited_ = false;
}

bool
GraphNode::IsLinkedTo(GraphNode* node) const
{
	for ( auto &i_link : links_ )
	{
		if ( i_link->node_a_ == node ) return true;
		if ( i_link->node_b_ == node ) return true;
	}

	return false;
}

bool
GraphNode::IsLinkedTo(GraphLink* link) const
{
	for (auto &i_link : links_)
	{
		if (i_link == link) return true;
	}

	return false;
}


int
GraphNode::CountLinks() const
{
	return (int)links_.size();
}




GraphNode*
GraphNode::GetUnvisitedLinkedNode() const
{
	int ii=0;
	for (auto &i_link : links_ )
	{
		if ( i_link->GetOppositeNode(this)->visited() == false ) 
			return i_link->GetOppositeNode(this);
	}

	return nullptr;
}



void
GraphNode::AddLink(GraphLink* link)
{
	assert( (link->node_a_== this) || (link->node_b_ == this) );

	if ( std::find(links_.begin(), links_.end(), link) == links_.end() )
	{
		links_.push_back(link);
	}
}

void
GraphNode::RemoveLink(GraphLink* link)
{
	auto &link_iter = std::find(links_.begin(), links_.end(), link);
	if (link_iter != links_.end() )
		links_.erase(link_iter);
}


void
GraphNode::ResetSCC()
{
	scc_index_ = -1;
	scc_lowindex_ = -1;
	scc_component_ = -1;
	visited_ = false;
}




////////////////////////////////////////////////////////
// Class Graph

void
Graph::CreateNodes(int num_nodes)
{
	for ( int i=0; i<num_nodes; i++ )
	{
		CreateNode();
	}
}

GraphNode*
Graph::CreateNode()
{
	std::unique_ptr<GraphNode> n(new GraphNode);
	nodes_.push_back( std::move(n) );

	return nodes_.back().get();
}



void
Graph::RemoveNode(GraphNode *n)
{
	// Remove links to n
	for (auto &i_n : nodes_)
	{
		if ( i_n->IsLinkedTo(n) )
		{
			RemoveLink(n, i_n.get());
		}
	}

	// Remove n
	for (unsigned int i = 0; i<nodes_.size(); i++)
	{
		if (nodes_[i].get() == n)
		{
			nodes_.erase(nodes_.begin() + i);
			break;
		}
	}
}

void
Graph::UnvisitAllNodes()
{
	for ( auto &i_node : nodes_ )
	{
		i_node->unvisit();
	}
}



GraphLink*
Graph::CreateLink(int a_node_id, int b_node_id)
{
	return CreateLink(nodes_[a_node_id].get(), nodes_[b_node_id].get());
}

GraphLink*
Graph::CreateLink(GraphNode* a, GraphNode* b)
{
	if ( IsLinked(a, b) ) return link(a, b);

	std::unique_ptr<GraphLink> link(new GraphLink);
	link->node_a_ = a;
	link->node_b_ = b;
	link->cost_ = a->CalculDistanceTo(b);
	
	links_.push_back(std::move(link));
	a->AddLink(links_.back().get());
	b->AddLink(links_.back().get());

	return links_.back().get();
}

void
Graph::RemoveLink(int a_node_id, int b_node_id)
{
	RemoveLink(nodes_[a_node_id].get(), nodes_[b_node_id].get());
}

void
Graph::RemoveLink(GraphNode* a, GraphNode* b)
{
	if ( !IsLinked(a, b) ) return;
	
	GraphLink *l = link(a, b);
	a->RemoveLink(l);
	b->RemoveLink(l);

	for ( int i=0; i<(int)links_.size(); i++ )
	{
		if ( links_[i].get() == l )
		{
			links_.erase(links_.begin()+i);
			break;
		}
	}
}


bool
Graph::IsLinked(int a_node_id, int b_node_id) const
{
	return IsLinked(nodes_[a_node_id].get(), nodes_[b_node_id].get());
}

bool
Graph::IsLinked(GraphNode* a, GraphNode* b) const
{
	if (a->IsLinkedTo(b)) return true;
	return false;
}

GraphLink*
Graph::link(int a_node_id, int b_node_id) const
{
	return link(nodes_[a_node_id].get(), nodes_[b_node_id].get());
}

GraphLink*
Graph::link(GraphNode* a, GraphNode* b) const
{
	for ( auto &i_link : links_ )
	{
		if ( i_link->node_a_ == a && i_link->node_b_ == b )
			return i_link.get();
		if ( i_link->node_b_ == a && i_link->node_a_ == b )
			return i_link.get();
	}

	return nullptr;
}

void 
Graph::GetNodeIndexOfLink(GraphLink* in, int &out_a_node_id, int &out_b_node_id)
{
	GraphNode* a_node = in->node_a();
	GraphNode* b_node = in->node_b();

	out_a_node_id = -1;
	out_b_node_id = -1;

	for ( unsigned int i=0; i<nodes_.size(); i++ )
	{
		if ( nodes_[i].get() == a_node ) out_a_node_id = i;
		if ( nodes_[i].get() == b_node ) out_b_node_id = i;

		if ( out_a_node_id != -1 && out_b_node_id != -1 ) return;
	}

	assert( out_a_node_id != -1 && out_b_node_id != -1 );
}



void
Graph::CalculStronglyConnectedComponent()
{
	// reset
	for (int i = 0; i<(int)nodes_.size(); i++)
	{
		nodes_[i]->ResetSCC();
	}
	scc_component_num_ = 0;
	scc_dfs_visit_index_ = 0;


	for (int i = 0; i<(int)nodes_.size(); i++)
	{
		if (nodes_[i]->scc_component_ == -1)
			CalculStronglyConnectedComponent(nodes_[i].get());
	}


	// print components info
	if (true)
	{
		std::vector<int> numbers_of_nodes_in_components;

		for (int i = 0; i<(int)nodes_.size(); i++)
		{
			int c_id = nodes_[i]->scc_component_;

			if (c_id < 0)
			{
				printf("node %d: %d\n", i, c_id);
				printf("index %d, lowindex %d\n", nodes_[i]->scc_index_, nodes_[i]->scc_lowindex_);
			}
			else
			{

				while ((int)numbers_of_nodes_in_components.size() <= c_id)
				{
					numbers_of_nodes_in_components.push_back(0);
				}

				numbers_of_nodes_in_components[c_id]++;
			}
		}


		int largest_ssc = 0;
		for (int i = 0; i<(int)numbers_of_nodes_in_components.size(); i++)
		{
			if (numbers_of_nodes_in_components[largest_ssc] < numbers_of_nodes_in_components[i])
				largest_ssc = i;
			//printf("component %d: %d\n", i, numbers_of_nodes_in_components[i]);
		}

		printf("largest_ssc: %d, num of nodes: %d\n", largest_ssc, numbers_of_nodes_in_components[largest_ssc]);
	}
}




///
// http://en.wikipedia.org/wiki/Tarjan's_strongly_connected_components_algorithm
void
Graph::CalculStronglyConnectedComponent(GraphNode *node)
{
	std::list<GraphNode*> scc_trace;

	std::stack<GraphNode*> dfs_stack;		// for vising all nodedepth first search
	dfs_stack.push(node);

	do
	{
		GraphNode* cur_node = dfs_stack.top();
		if (!cur_node->visited_)
		{
			cur_node->visited_ = true;

			cur_node->scc_index_ = scc_dfs_visit_index_;
			scc_dfs_visit_index_++;
			cur_node->scc_lowindex_ = cur_node->scc_index_;
			scc_trace.push_back(cur_node);
		}


		GraphNode *unvisited_next = cur_node->GetUnvisitedLinkedNode();

		if (unvisited_next != nullptr)
		{
			dfs_stack.push(unvisited_next);
		}

		else
		{
			for ( auto &i_link : cur_node->links_ )
			{
				GraphNode *next = i_link->GetOppositeNode(cur_node);
				if (std::find(scc_trace.begin(), scc_trace.end(), next) != scc_trace.end())
				{
					int next_min = std::min(next->scc_index_, next->scc_lowindex_);
					cur_node->scc_lowindex_ = std::min(next_min, cur_node->scc_lowindex_);
				}
			}


			//
			if (cur_node->scc_lowindex_ == cur_node->scc_index_)
			{
				GraphNode *tmp;
				do
				{
					tmp = scc_trace.back();
					tmp->scc_component_ = scc_component_num_;
					scc_trace.pop_back();
				} while (cur_node != tmp);

				scc_component_num_++;
			}


			dfs_stack.pop();
		}

	} while (!dfs_stack.empty());


}


void
Graph::LeaveLargestSCC()
{
	// count numbers of nodes in each component.
	std::vector<int> numbers_of_nodes_in_components;
	for (int i = 0; i<(int)nodes_.size(); i++)
	{
		int c_id = nodes_[i]->scc_component_;

		while ((int)numbers_of_nodes_in_components.size() <= c_id)
		{
			numbers_of_nodes_in_components.push_back(0);
		}

		numbers_of_nodes_in_components[c_id]++;
	}

	// the largest component
	int largest_component = 0;
	for (int i = 0; i<(int)numbers_of_nodes_in_components.size(); i++)
	{
		if (numbers_of_nodes_in_components[largest_component]
			< numbers_of_nodes_in_components[i])
		{
			largest_component = i;
		}
	}

	// Gather the nodes of Non-largest components
	std::vector<GraphNode*> non_largest_coms;
	for ( auto &i_n : nodes_ )
	{
		if ( i_n->scc_component_ != largest_component )
		{
			non_largest_coms.push_back(i_n.get());
		}
	}

	// remove the nodes of Non-largest components.
	for (auto &i_n : non_largest_coms)
	{
		RemoveNode(i_n);
	}


}


void
Graph::RemoveLargestSCC()
{
	// count numbers of nodes in each component.
	std::vector<int> numbers_of_nodes_in_components;
	for (int i = 0; i<(int)nodes_.size(); i++)
	{
		int c_id = nodes_[i]->scc_component_;

		while ((int)numbers_of_nodes_in_components.size() <= c_id)
		{
			numbers_of_nodes_in_components.push_back(0);
		}

		numbers_of_nodes_in_components[c_id]++;
	}

	// the largest component
	int largest_component = 0;
	for (int i = 0; i<(int)numbers_of_nodes_in_components.size(); i++)
	{
		if (numbers_of_nodes_in_components[largest_component]
			< numbers_of_nodes_in_components[i])
		{
			largest_component = i;
		}
	}

	// Gather the nodes of largest components
	std::vector<GraphNode*> largest_coms;
	for ( auto &i_n : nodes_ )
	{
		if ( i_n->scc_component_ == largest_component )
		{
			largest_coms.push_back(i_n.get());
		}
	}

	// remove the nodes of Non-largest components.
	for (auto &i_n : largest_coms)
	{
		RemoveNode(i_n);
	}


}



///////////////////////////////////////
// Dijkstra Algorithm
void
Graph::CalculDijkstraShortest(GraphNode *start_node)
{
	// Initialize
	dijkstra_result_back_traces_.clear();
	dijkstra_acc_costs_.clear();
	UnvisitAllNodes();

	for ( auto &i_node : nodes_ )
	{
		dijkstra_result_back_traces_[i_node.get()] = nullptr;
		dijkstra_acc_costs_[i_node.get()] = std::numeric_limits<double>::max();
	}
	dijkstra_result_back_traces_[start_node] = nullptr;
	dijkstra_acc_costs_[start_node] = 0;

	std::queue<GraphNode*> bfs;	// breath first searh queue
	bfs.push(start_node);

	while ( !bfs.empty() )
	{
		GraphNode* cur_node = bfs.front();
		bfs.pop();
		cur_node->visit();

		for ( GraphLink *i_link : cur_node->links() )
		{
			// update dijkstra costs
			GraphNode* to_node = i_link->GetOppositeNode(cur_node);
			if ( to_node->visited() ) continue;

			double new_cost = dijkstra_acc_costs_[cur_node] + i_link->cost();
			if ( new_cost < dijkstra_acc_costs_[to_node] )
			{
				dijkstra_acc_costs_[to_node] = new_cost;
				dijkstra_result_back_traces_[to_node] = cur_node;
			}


			// breath first search
			bfs.push(to_node);
		}

	}

	assert(dijkstra_result_traces_.size()==nodes_.size());
}


void
Graph::GetDijkstraShortestPathTo(GraphNode *to, std::vector<GraphNode*> &out_path)
{
	out_path.clear();
	while (1)
	{
		out_path.push_back(to);
		to = dijkstra_result_back_traces_[to];
		if ( to == nullptr ) break;
	}
	
	std::reverse(out_path.begin(), out_path.end());
}

};