/**
 * This module contains the class computing the Kinematics of the Scara Robot
 */

#ifndef HW1_BASIC_SEARCH_DIJKSTRA_HPP
#define HW1_BASIC_SEARCH_DIJKSTRA_HPP

#include "Node.hpp"
#include <vector>
#include <utility>
using namespace std;


namespace motion_planning
{
	class Dijkstra
	{
	public:
		Dijkstra();

		~Dijkstra() = default;

		bool search( const Eigen::MatrixXi& grid,
					 const Eigen::Vector2i& start,
					 const Eigen::Vector2i& goal,
					 vector<Eigen::Vector2i>& path,
					 int& steps);

	private:

		MatrixXNode m_graph;			// input map

		int m_total_rows;

		int m_total_cols;

		void initialize_graph(const Eigen::MatrixXi& grid,
							  const Eigen::Vector2i& start,
							  const Eigen::Vector2i& goal);

		void find_valid_neighbors(const shared_ptr<Node> current_node,
								  vector<pair<shared_ptr<Node>,int>>& neighbors_w_list);

		bool search_path(const Eigen::Vector2i& start,
						 const Eigen::Vector2i& goal,
						 vector<Eigen::Vector2i>& path);

	};

}  // namespace motion_planning

#endif	// #ifndef HW1_BASIC_SEARCH_DIJKSTRA_HPP
