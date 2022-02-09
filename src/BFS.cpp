/**
 * This module contains BFS, DFS, Djikstras, A* algorithms
 */

#include "BFS.hpp"
#include "Utils.hpp"
#include <queue>
#include <algorithm>

namespace motion_planning
{

	BFS::BFS()	: 	m_graph(),
					m_total_rows(0),
					m_total_cols(0)
	{

	}


	bool BFS::search(const Eigen::MatrixXi& grid,
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
		queue <shared_ptr<Node>> Q;
		Q.push(start_node);


		// [4] Update each nodes' parents and distance from start using BFS
		auto current_node = Q.front();
		vector<pair<shared_ptr<Node>,int>> neighbors_w;
		int row=0, col=0;

		while ( (!Q.empty()) && (current_node != goal_node) )
		{
			current_node = Q.front();
			Q.pop();

			current_node->visited = true;
			steps += 1;

			row = current_node->row;
			col = current_node->col;

			neighbors_w.clear();
			Utils::find_valid_neighbors(m_graph, current_node, neighbors_w);

			for (auto& node_w : neighbors_w)
			{
				auto node = node_w.first;
				auto w = node_w.second;		// not useful for this algo

				if ( !node->visited )
				{
					node->cost 	 	= current_node->cost + 1;
					node->parent 	= current_node;
					node->visited 	= true;
					Q.push(node);
				}
			}
		}

		// [5] Using updated graph's info, find the path from start to goal
		return Utils::search_path(m_graph, start, goal, path);
	}

}	// namespace motion_planning
