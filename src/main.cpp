/**
 * This module contains the main function
 */

//#include "ROSLogistics.hpp"
#include "BFS.hpp"
#include "DFS.hpp"
#include "Dijkstra.hpp"
#include "AStar.hpp"
#include <memory>
using namespace std;

int main(int argc, char **argv)
{
//	ros::init(argc, argv, "hw1_basic_search_algo", ros::init_options::AnonymousName);

	Eigen::MatrixXi grid(10,10);
	grid << 0,	0,	0,	0,	0,	0,	0,	0,	1,	0,
			0,	1,	1,	1,	1,	0,	0,	0,	1,	0,
			0,	0,	0,	0,	0,	0,	0,	0,	1,	0,
			0,	0,	0,	1,	1,	1,	1,	1,	1,	0,
			0,	1,	0,	1,	0,	0,	0,	0,	0,	0,
			0,	1,	0,	1,	0,	1,	1,	1,	1,	0,
			0,	1,	0,	0,	0,	1,	1,	1,	1,	0,
			0,	1,	1,	1,	0,	0,	0,	1,	1,	0,
			0,	0,	0,	0,	0,	0,	0,	1,	0,	0,
			0,	0,	0,	0,	0,	0,	0,	0,	0,	0;

	const Eigen::Vector2i start {0,0};
	const Eigen::Vector2i goal {6,9};
	vector<Eigen::Vector2i> path;
	int steps;


	cout << "-------------BFS-------------" << endl;
	motion_planning::BFS bfs;
	bool result = bfs.search(grid, start, goal, path, steps);

	if (result)
	{
		cout << "Steps: " << steps << endl;
		for (auto& i : path)
		{
			cout << "[" << i(0) << "," << i(1) << "], ";
		}
		cout << endl;
	}
	else
	{
		cout << "Path not found" << endl;
	}

	cout << "-------------DFS-------------" << endl;
	motion_planning::DFS dfs;
	result = dfs.search(grid, start, goal, path, steps);

	if (result)
	{
		cout << "Steps: " << steps << endl;
		for (auto& i : path)
		{
			cout << "[" << i(0) << "," << i(1) << "], ";
		}
		cout << endl;
	}
	else
	{
		cout << "Path not found" << endl;
	}


	cout << "-------------Dijkstra-------------" << endl;
	motion_planning::Dijkstra dijkstra;
	result = dijkstra.search(grid, start, goal, path, steps);

	if (result)
	{
		cout << "Steps: " << steps << endl;
		for (auto& i : path)
		{
			cout << "[" << i(0) << "," << i(1) << "], ";
		}
		cout << endl;
	}
	else
	{
		cout << "Path not found" << endl;
	}

	cout << "-------------A*-------------" << endl;
	motion_planning::AStar astar;
	result = astar.search(grid, start, goal, path, steps);

	if (result)
	{
		cout << "Steps: " << steps << endl;
		for (auto& i : path)
		{
			cout << "[" << i(0) << "," << i(1) << "], ";
		}
		cout << endl;
	}
	else
	{
		cout << "Path not found" << endl;
	}

//	/* This starts the ROS server to serve different algo etc */
//	motion_planning::ROS_Logistics rosObj();

    return 0;
}

