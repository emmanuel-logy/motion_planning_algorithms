/**
 * This module contains the class computing the Kinematics of the Scara Robot
 */

#ifndef HW1_BASIC_SEARCH_DFS_HPP
#define HW1_BASIC_SEARCH_DFS_HPP

#include "Node.hpp"
#include <vector>
#include <utility>
using namespace std;


namespace motion_planning
{
	class DFS
	{
	public:
		DFS();

		~DFS() = default;

		bool search( const Eigen::MatrixXi& grid,
					 const Eigen::Vector2i& start,
					 const Eigen::Vector2i& goal,
					 vector<Eigen::Vector2i>& path,
					 int& steps);

	private:

		MatrixXNode m_graph;			// input map

		int m_total_rows;

		int m_total_cols;

		bool m_goal_hit;

		void dfs_recursive_visit(shared_ptr<Node> current_node, shared_ptr<Node> goal_node, int& steps);

	};

}  // namespace motion_planning

#endif	// #ifndef HW1_BASIC_SEARCH_DFS_HPP
