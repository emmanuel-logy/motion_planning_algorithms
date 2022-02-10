/**
 * This module contains the class computing the Kinematics of the Scara Robot
 */

#ifndef HW1_BASIC_SEARCH_UTILS_HPP
#define HW1_BASIC_SEARCH_UTILS_HPP

#include "Node.hpp"
using namespace std;


namespace motion_planning
{
	class Utils
	{
	public:

		static void initialize_graph(const Eigen::MatrixXi& grid,
									 MatrixXNode& graph);


		static void find_valid_neighbors(const MatrixXNode& graph,
										 const shared_ptr<Node> current_node,
										 vector<pair<shared_ptr<Node>,int>>& neighbors_w_list);


		static bool generate_path(const MatrixXNode& graph,
								const Eigen::Vector2i& start,
								const Eigen::Vector2i& goal,
								vector<Eigen::Vector2i>& path);

	private:

	};

}  // namespace motion_planning

#endif	// #ifndef HW1_BASIC_SEARCH_UTILS_HPP
