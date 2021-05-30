#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Path.h"
#include "path_planning/input_params.h"

#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/Path.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/geometric/PathGeometric.h"
#include <sstream>
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/config.h"
#include <iostream>
#include <utility>

#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/samplers/UniformValidStateSampler.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/ScopedState.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <boost/math/constants/constants.hpp>
namespace ob = ompl::base;
namespace og = ompl::geometric;
ros::Publisher path_pub;

class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr &si) : ob::StateValidityChecker(si) {}

    bool isValid(const ob::State *state) const
    {
        return this->clearance(state) > 0.0;
    }

    double clearance(const ob::State *state) const
    {
        const ob::RealVectorStateSpace::StateType *state2D =
            state->as<ob::RealVectorStateSpace::StateType>();

        double x = state2D->values[0];
        double y = state2D->values[1];

        return sqrt((x - 0.5) * (x - 0.5) + (y - 0.5) * (y - 0.5)) - 0.25;
    }
};
ob::OptimizationObjectivePtr getObjective(const ob::SpaceInformationPtr &si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}
ob::OptimizationObjectivePtr getThreshold(const ob::SpaceInformationPtr &si, double threshold)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ob::Cost(threshold));
    return obj;
}
std::pair<ob::ProblemDefinitionPtr, ob::SpaceInformationPtr> construct(double start_x, double start_y, double goal_x, double goal_y, double threshold)
{
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));
    space->as<ob::RealVectorStateSpace>()->setBounds(0.0, 100.0);
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
    si->setup();

    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_x;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_y;

    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_x;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_y;

    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->setStartAndGoalStates(start, goal);
    //pdef->setOptimizationObjective(getObjective(si));
    pdef->setOptimizationObjective(getThreshold(si, threshold));
    return std::make_pair(pdef, si);
}

bool execute(path_planning::input_params::Request &req,
             path_planning::input_params::Response &res)
{
    double dyn_range = 5, dyn_threshold = 75;
    double computation_time = 2.0;
    auto pdef = construct(req.start_x, req.start_y, req.end_x, req.end_y, dyn_threshold);
    og::RRTstar *rrt = new og::RRTstar(pdef.second);
    rrt->setRange(dyn_range);

    ob::PlannerPtr optimizingPlanner(rrt);

    optimizingPlanner->setProblemDefinition(pdef.first);
    optimizingPlanner->setup();
    double initial_time = ros::Time::now().toSec();
    ob::PlannerStatus solved = optimizingPlanner->solve(computation_time);
    res.elapsedTime = ros::Time::now().toSec() - initial_time;
    if (solved)
    {
        std::cout
            << optimizingPlanner->getName()
            << " found a solution of length "
            << pdef.first->getSolutionPath()->length()
            << " with an optimization objective value of "
            << pdef.first->getSolutionPath()->cost(pdef.first->getOptimizationObjective()) << std::endl
            << pdef.first->getSolutionCount() << std::endl;
        ob::PlannerSolution *foundPaths = new ob::PlannerSolution(pdef.first->getSolutionPath());

        pdef.first->getSolutionPath().get()->print(std::cout);
        og::PathGeometric path(dynamic_cast<const og::PathGeometric &>(*pdef.first->getSolutionPath()));
        const std::vector<ob::State *> &states = path.getStates();
        ob::State *state;
        nav_msgs::Path nav_path;
        nav_path.header.frame_id = "map";
        geometry_msgs::PoseStamped pose;
        for (size_t i = 0; i < states.size(); ++i)
        {
            state = states[i]->as<ob::State>();
            //nav_path.poses
            double x, y;
            pose.pose.position.x = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
            pose.pose.position.y = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
            pose.header.frame_id = "base_link";
            nav_path.poses.push_back(pose);
            //std::cout << x << " " << y << std::endl;
        }

        if (pdef.first->getSolutionPath()->length() > dyn_threshold)
        {
            res.isFound = false;
            res.elapsedTime = 0.0;
            return true;
        }

        path_pub.publish(nav_path);
        res.isFound = true;
        return true;
    }
    std::cout << "No solution found." << std::endl;
    res.elapsedTime = 0.0;
    res.isFound = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;
    path_pub = nh.advertise<nav_msgs::Path>("Path_Publisher", 1000);
    visualization_msgs::Marker obstacles[10];
    for (int i = 0; i < 10; i++)
    {
        obstacles[i].header.frame_id = "base_link";
        obstacles[i].header.stamp = ros::Time();
        obstacles[i].ns = "obstacle";
        obstacles[i].id = i;
        obstacles[i].type = visualization_msgs::Marker::SPHERE;
        obstacles[i].action = visualization_msgs::Marker::ADD;
        obstacles[i].pose.position.x = 1;
        obstacles[i].pose.position.y = 1;
        obstacles[i].pose.position.z = 0;
        obstacles[i].pose.orientation.x = 0.0;
        obstacles[i].pose.orientation.y = 0.0;
        obstacles[i].pose.orientation.z = 0.0;
        obstacles[i].pose.orientation.w = 1.0;
        obstacles[i].scale.x = 1;
        obstacles[i].scale.y = 0.1;
        obstacles[i].scale.z = 0.1;
        obstacles[i].color.a = 1.0;
        obstacles[i].color.r = 0.0;
        obstacles[i].color.g = 1.0;
        obstacles[i].color.b = 0.0;
    }

    ros::Publisher obstacles_pub = nh.advertise<visualization_msgs::Marker>("Obstacles_Publisher", 1000);
    //ros::Rate loop_rate(10);
    ros::ServiceServer service = nh.advertiseService("input_params", execute);
    ROS_INFO("path_planner node has been started.");
    ros::spin();

    return 0;
}
