/**
 * This module contains Dijkstra, DFS, Djikstras, A* algorithms
 */

#include "Utils.hpp"
#include <algorithm>

namespace motion_planning
{

	void Utils::initialize_graph(const Eigen::MatrixXi& grid,
								 MatrixXNode& graph)
	{
		int rows = grid.rows();
		int cols = grid.cols();

		// Fill the graph matrix with Nodes corresponding to grid information
		for (int i=0; i<rows; ++i)
		{
			for (int j=0; j<cols; ++j)
			{
				graph(i,j) = make_shared<Node>(i, j, grid(i,j));
			}
		}
	}


	void Utils::find_valid_neighbors(const MatrixXNode& graph,
									 const shared_ptr<Node> current_node,
								     vector<pair<shared_ptr<Node>,int>>& neighbors_w_list)
	{
		neighbors_w_list.clear();

		int total_rows = graph.rows();
		int total_cols = graph.cols();

		int row = current_node->row;
		int col = current_node->col;

		// Right neighbor
		if ( (col+1)< total_cols && !graph(row, col+1)->obs)
		{
			auto R = graph(row, col+1);
			neighbors_w_list.push_back(make_pair(R, current_node->edge_w[0]));
		}

		// Down neighbor
		if ( (row+1)< total_rows && !graph(row+1, col)->obs)
		{
			auto D = graph(row+1, col);
			neighbors_w_list.push_back(make_pair(D, current_node->edge_w[1]));
		}

		// Left neighbor
		if ( (col-1)> 0 && !graph(row, col-1)->obs)
		{
			auto L = graph(row, col-1);
			neighbors_w_list.push_back(make_pair(L, current_node->edge_w[2]));
		}

		// Top neighbor
		if ( (row-1)> 0 && !graph(row-1, col)->obs)
		{
			auto T = graph(row-1, col);
			neighbors_w_list.push_back(make_pair(T, current_node->edge_w[3]));
		}
	}


	bool Utils::search_path(const MatrixXNode& graph,
							const Eigen::Vector2i& start,
						    const Eigen::Vector2i& goal,
						    vector<Eigen::Vector2i>& path)
	{
		path.clear();

		auto current_node = graph(goal(0), goal(1));

		// If parent doesn't exist for goal node, then path was not found
		if (current_node->parent == nullptr)
		{
			path.clear();
			return false;
		}
		else
		{
			// Starting from goal node, move to start node using parent info to find the path
			while (current_node != graph(start(0), start(1)))
			{
				path.push_back({current_node->row, current_node->col});
				current_node = graph(current_node->parent->row, current_node->parent->col);
			}
			path.push_back({current_node->row, current_node->col});		// push start node also

			reverse(path.begin(), path.end());							// get path from start to goal
			return true;
		}
	}

}	// namespace motion_planning
