#include "RRT.h"

node *RRT::getNearestNode(const std::vector<double> &random_position)
{
    double min_distance = std::numeric_limits<double>::max();
    int mark = -1;
    for (std::size_t i = 0; i < NodeList.size(); i++)
    {
        double dis_sqrt = std::pow(NodeList[i]->getX() - random_position[0], 2) + std::pow(NodeList[i]->getY() - random_position[1], 2);
        if (dis_sqrt < min_distance)
        {
            min_distance = dis_sqrt;
            mark = i;
        }
    }
    return NodeList[mark];
}

bool RRT::CollisionCheck(node *CurrentNode)
{
    for (std::size_t i = 0; i < ObstacleList_2D.size(); i++)
    {
        switch (ObstacleList_2D.size())
        {
        case 3:
            double dis = std::pow(CurrentNode->getX() - ObstacleList_2D[i][0], 2) + std::pow(CurrentNode->getY() - ObstacleList_2D[i][1], 2);
            if (dis - std::pow(ObstacleList_2D[i][2], 2) > 1e-5)
                return true;
            break;
        case 4:
            if (std::abs(CurrentNode->getX() - ObstacleList_2D[i][0]) < ObstacleList_2D[i][2] && std::abs(CurrentNode->getY() - ObstacleList_2D[i][1]) < ObstacleList_2D[i][3])
                return true;
            break;
        case 6:
            break;
        default:
            std::cerr << "Wrong Obstacle description!\n";
            return true;
        }
    }
    return false;
}

std::vector<node *> RRT::planning()
{
    NodeList.clear();
    if (CollisionCheck(StartNode))
    {
        std::cerr << "The start point is in a obstacle!\n";
        return NodeList;
    }
    if (CollisionCheck(GoalNode))
    {
        std::cerr << "The goal point is in a obstacle!\n";
        return NodeList;
    }
    NodeList.push_back(StartNode);
    std::vector<double> random_position;
    double randX, randY;
    while (NodeList.back() != GoalNode)
    {
        if (goal_dis(goal_gen) > goal_sample_rate)
        {
            randX = area_dis(goal_gen);
            randY = area_dis(goal_gen);
            random_position.push_back(randX);
            random_position.push_back(randY);
        }
        else
        {
            random_position.push_back(GoalNode->getX());
            random_position.push_back(GoalNode->getY());
        }
        node *NearestNode = getNearestNode(random_position);
        double dis = std::sqrt(std::pow(NearestNode->getX() - random_position[0], 2) + std::pow(NearestNode->getY() - random_position[1], 2));
        node *NextNode = new node((random_position[0] - NearestNode->getX()) / dis * step, (random_position[1] - NearestNode->getY()) / dis * step);
        NextNode->setParent(NearestNode);

        if (CollisionCheck(NextNode))
            continue;
        else
            NodeList.push_back(NextNode);
    }
    return NodeList;
}
