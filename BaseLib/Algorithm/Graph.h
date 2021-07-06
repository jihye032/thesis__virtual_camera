#pragma once

#include <memory>
#include <vector>
#include <map>

namespace mg
{

class GraphLink;

class GraphNode
{
	friend class Graph;

public:
	GraphNode();

	bool IsLinkedTo(GraphNode* node) const;
	bool IsLinkedTo(GraphLink* link) const;
	int CountLinks() const;

	GraphNode* GetUnvisitedLinkedNode() const;
	const std::vector<GraphLink*>& links() const { return links_; }

	bool visited() const { return visited_; }
	void visit() { visited_ = true; }
	void unvisit() { visited_ = false; }

	void* data() const { return data_; }
	void data(void* d) { data_ = d; }

	virtual double CalculDistanceTo(const GraphNode *b) const
	{
		return 0;
	}

protected:
	/*
	links_ contains the links from this node to others.
	These are used just for an easy access to the links and linked nodes.
	Duplicated information about links will be written in GraphLink and Graph classes.
	For the consistancy of the duplicated data links_ musted be managed only in Graph class
	by callinbg AddLink and RemoveLink functions. 
	*/
	std::vector<GraphLink*> links_;
	void AddLink(GraphLink* link);
	void RemoveLink(GraphLink* link);
	// void RemoveLink(GraphNode* node);


private:
	bool visited_;

	// for finding strongly connected components.
	void ResetSCC();
	int scc_index_;
	int scc_lowindex_;
	int scc_component_;

protected:
	void* data_;
};

class GraphLink
{
	friend class Graph;
	friend class GraphNode;

public:
	GraphLink()
	{
		node_a_ = 0;
		node_b_ = 0;
		cost_ = 0;
	}

	double cost() const { return cost_; }
	void cost(double c) { cost_ = c; }

	GraphNode* node_a() const { return node_a_; }
	GraphNode* node_b() const { return node_b_; }

	GraphNode* GetOppositeNode(const GraphNode* a) const 
	{
		if ( a == node_a_ ) return node_b_;
		return node_a_;
	}

protected:
	GraphNode* node_a_;
	GraphNode* node_b_;
	double cost_;
};
	



class Graph
{
	

public:
	Graph();
	virtual ~Graph();
	virtual GraphNode* CreateNode();
	void CreateNodes(int num_nodes);

	int CountNodes() const { return (int)nodes_.size(); }
	void RemoveNode(GraphNode *n);
	void UnvisitAllNodes();
	GraphNode* node(int i) { return nodes_[i].get(); }
	const std::vector< std::unique_ptr<GraphNode> >& nodes() const { return nodes_; }

	virtual GraphLink* CreateLink(GraphNode* a, GraphNode *b);
	virtual GraphLink* CreateLink(int a_node_id, int b_node_id);
	void RemoveLink(GraphNode* a, GraphNode* b);
	void RemoveLink(int a_node_id, int b_node_id);
	GraphLink* link(GraphNode* a, GraphNode* b) const;
	GraphLink* link(int a_node_id, int b_node_id) const;
	const std::vector< std::unique_ptr<GraphLink> >& links() const { return links_; }
	void GetNodeIndexOfLink(GraphLink* in, int &out_a_node_id, int &out_b_node_id);

	bool IsLinked(GraphNode* a, GraphNode* b) const;
	bool IsLinked(int a_node_id, int b_node_id) const;
	
protected:
	std::vector< std::unique_ptr<GraphNode> > nodes_;
	std::vector< std::unique_ptr<GraphLink> > links_;



	////////////////////////////////////////////////////////////////
	// Graph Algorithms
	// scc - Tarjan's Algorithm
public:
	void CalculStronglyConnectedComponent();
	void CalculStronglyConnectedComponent(GraphNode* node);
	void LeaveLargestSCC();
	void RemoveLargestSCC();
	
protected:
	int scc_component_num_;
	int scc_dfs_visit_index_;


public:
	// Dijkstra Algorithm
	void CalculDijkstraShortest(GraphNode *start);
	void GetDijkstraShortestPathTo(GraphNode *to, std::vector<GraphNode*> &out_pat);
	std::map<GraphNode*, GraphNode*> dijkstra_result_back_traces_;
	std::map<GraphNode*, double> dijkstra_acc_costs_;

	
};



};

