#include <cmath>
#include <algorithm>
#include <vector>
#include <random>
#include <iostream>
#include <ctime>
#include <limits>

class node
{
private:
	double x, y;
	std::vector<double> pathX, pathY;
	node *Parent;
	double cost;

public:
	node(double x_, double y_) : x(x_), y(y_), Parent(nullptr), cost(0.0){};
	double getX() { return x; };
	double getY() { return y; };
	void setParent(node *Parent_) { Parent = Parent_; };
	node *getParent() { return Parent; };
};

class RRT
{
private:
	node *StartNode, *GoalNode;
	std::vector<std::vector<double>> ObstacleList_2D; // Circle: center_x, center_y, R; Rectangle: center_x, center_y, long, high;Triangle: A_x, A_y, B_x, B_y, C_x, C_y
	std::vector<node *> NodeList;
	double step;
	int goal_sample_rate;

	// 随机数生成器
	std::random_device goal_rd;
	std::mt19937 goal_gen;
	std::uniform_int_distribution<int> goal_dis;

	std::random_device area_rd;
	std::mt19937 area_gen;
	std::uniform_real_distribution<double> area_dis;

public:
	RRT(node *StartNode_, node *GoalNode_, const std::vector<std::vector<double>> ObstacleList_, double step_, int goal_sample_rate_)
		: StartNode(StartNode_),
		  GoalNode(GoalNode_),
		  ObstacleList_2D(ObstacleList_),
		  step(step_),
		  goal_sample_rate(goal_sample_rate_),
		  goal_dis(std::uniform_int_distribution<int>(0, 100)),
		  area_dis(std::uniform_real_distribution<double>(0, 15)),
		  goal_gen(goal_rd()),
		  area_gen(area_rd){};
	~RRT();
	node *getNearestNode(const std::vector<double> &random_position);
	bool CollisionCheck(node *CurrentNode);
	std::vector<node *> planning();
};
