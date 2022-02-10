/**
 * This module contains BFS, DFS, Djikstras, A* algorithms
 */

#include "DFS.hpp"
#include "Utils.hpp"
#include <algorithm>

namespace motion_planning
{

	DFS::DFS()	: 	m_graph(),
					m_total_rows(0),
					m_total_cols(0),
					m_goal_hit(false)
	{

	}


	bool DFS::search(const Eigen::MatrixXi& grid,
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


		// [2] Start DFS recursive logic
		auto start_node = m_graph(start(0), start(1));
		auto goal_node = m_graph(goal(0), goal(1));
		dfs_recursive_visit(start_node, goal_node, steps);


		// [3] Using updated graph's info, find the path from start to goal
		return Utils::generate_path(m_graph, start, goal, path);
	}


	void DFS::dfs_recursive_visit(shared_ptr<Node> current_node, shared_ptr<Node> goal_node, int& steps)
	{
		current_node->visited = true;
		if (!m_goal_hit)
			steps += 1;

		vector<pair<shared_ptr<Node>,int>> neighbors_w;
		Utils::find_valid_neighbors(m_graph, current_node, neighbors_w);

		for (auto node_w : neighbors_w)
		{
			auto node = node_w.first;
			auto w = node_w.second;				// not useful for this algo

			if ( node && !node->visited )
			{
				node->parent = current_node;

				if (node == goal_node)
				{
					m_goal_hit = true;
					steps += 1;					// since we need to count goal_node also
				}

				dfs_recursive_visit(node, goal_node, steps);
			}
		}
	}

}	// namespace motion_planning
