/**
 * This module contains BFS, DFS, Djikstras, A* algorithms
 */

#include "BFS.hpp"
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
		initialize_graph(grid, start, goal);


		// [2] Start & Goal node
		auto start_node = m_graph(start(0), start(1));
		auto goal_node 	= m_graph(goal(0), goal(1));


		// [3] Queuing the nodes to visit in the loop
		queue <shared_ptr<Node>> Q;
		Q.push(start_node);

		// [4] Update each nodes' parents and distance from start using BFS
		auto current_node = Q.front();
		vector<shared_ptr<Node>> neighbors;
		int row=0, col=0;

		while ( (!Q.empty()) && (current_node != goal_node) )
		{
			current_node = Q.front();
			Q.pop();

			row = current_node->row;
			col = current_node->col;

			neighbors.clear();
			find_valid_neighbors(current_node, neighbors);

			for (auto& node : neighbors)
			{
				if ( !node->visited )
				{
					node->cost 	 	= current_node->cost + 1;
					node->parent 	= current_node;
					node->visited 	= true;
					Q.push(node);
				}
			}

			current_node->visited = true;
			steps += 1;
		}

		// [5] Using updated graph's info, find the path from start to goal
		return search_path(start, goal, path);
	}


	void BFS::initialize_graph(const Eigen::MatrixXi& grid,
							   const Eigen::Vector2i& start,
							   const Eigen::Vector2i& goal)
	{
		m_total_rows = grid.rows();
		m_total_cols = grid.cols();
		m_graph = MatrixXNode(m_total_rows, m_total_cols);		// empty graph matrix is ready

		// Fill the graph matrix with Nodes corresponding to grid information
		for (int i=0; i<m_total_rows; ++i)
		{
			for (int j=0; j<m_total_cols; ++j)
			{
				m_graph(i,j) = make_shared<Node>(i, j, grid(i,j));
			}
		}
	}


	void BFS::find_valid_neighbors(const shared_ptr<Node>& current_node,
							 	   vector<shared_ptr<Node>>& neighbors_list)
	{
		neighbors_list.clear();

		int row = current_node->row;
		int col = current_node->col;

		// Right neighbor
		if ( (col+1)< m_total_cols && !m_graph(row, col+1)->obs)
			neighbors_list.push_back(m_graph(row, col+1));

		// Bottom neighbor
		if ( (row+1)< m_total_rows && !m_graph(row+1, col)->obs)
			neighbors_list.push_back(m_graph(row+1, col));

		// Left neighbor
		if ( (col-1)> 0 && !m_graph(row, col-1)->obs)
			neighbors_list.push_back(m_graph(row, col-1));

		// Top neighbor
		if ( (row-1)> 0 && !m_graph(row-1, col)->obs)
			neighbors_list.push_back(m_graph(row-1, col));

	}


	bool BFS::search_path(const Eigen::Vector2i& start,
						  const Eigen::Vector2i& goal,
						  vector<Eigen::Vector2i>& path)
	{
		path.clear();

		auto current_node = m_graph(goal(0), goal(1));

		// If parent doesn't exist for goal node, then path was not found
		if (current_node->parent == nullptr)
		{
			path.clear();
			return false;
		}
		else
		{
			// Starting from goal node, move to start node using parent info to find the path
			while (current_node != m_graph(start(0), start(1)))
			{
				path.push_back({current_node->row, current_node->col});
				current_node = m_graph(current_node->parent->row, current_node->parent->col);
			}
			path.push_back({current_node->row, current_node->col});		// push start node also

			reverse(path.begin(), path.end());							// get path from start to goal
			return true;
		}
	}

}	// namespace motion_planning
