/**
 * This module contains AStar, AStar, Djikstras, A* algorithms
 */

#include "AStar.hpp"
#include "Utils.hpp"
#include <queue>
#include <algorithm>
#include <cmath>

namespace motion_planning
{

	// To order the priority queue in the ascedning order
	class cmp {
	public:
		bool operator() (const shared_ptr<Node> n1, const shared_ptr<Node> n2)
		{
			return (n1->cost  >  n2->cost);
		}
	};

	AStar::AStar()	: 	m_graph(),
						m_total_rows(0),
						m_total_cols(0)
	{

	}


	bool AStar::search(const Eigen::MatrixXi& grid,
					   const Eigen::Vector2i& start,
					   const Eigen::Vector2i& goal,
					   vector<Eigen::Vector2i>& path,
					   int& steps)
	{
		cout << "searching . . ." << endl;

		// [0] Clearing garbage values in output variables
		path.clear();
		steps = 0;


		// [1] Converting the input grid into graph of Nodes and initializing each node
		m_total_rows = grid.rows();
		m_total_cols = grid.cols();
		m_graph = MatrixXNode(m_total_rows, m_total_cols);		// empty graph matrix is ready
		Utils::initialize_graph(grid, m_graph);


		// [2] Start & Goal node
		auto start_node = m_graph(start(0), start(1));
		auto goal_node 	= m_graph(goal(0), goal(1));


		// [3] Queuing the nodes to visit in the loop
		priority_queue <shared_ptr<Node>,
						vector<shared_ptr<Node>>,
						cmp> Q;
		Q.push(start_node);

		// [4] Update each nodes' parents and distance from start using AStar
		auto current_node = Q.top();
		current_node->cost = 0;
		vector<pair<shared_ptr<Node>,int>> neighbors_w;
		int row=0, col=0;

		while ( (!Q.empty()) && (current_node != goal_node) )
		{
			current_node = Q.top();
			Q.pop();

			if ( !current_node->visited )
			{
				current_node->visited = true;
				steps += 1;

				row = current_node->row;
				col = current_node->col;

				neighbors_w.clear();
				Utils::find_valid_neighbors(m_graph, current_node, neighbors_w);

				for (auto& node_w : neighbors_w)
				{
					auto node = node_w.first;
					auto w = node_w.second;

					auto d = current_node->cost + w;
					d += abs(goal_node->row - node->row) + abs(goal_node->col - node->col);
					if ( !node->visited && d < node->cost )
					{
						node->cost 	 	= d;
						node->parent 	= current_node;
						Q.push(node);
					}
				}
			}
		}

		// [5] Using updated graph's info, find the path from start to goal
		return Utils::search_path(m_graph, start, goal, path);
	}


}	// namespace motion_planning
