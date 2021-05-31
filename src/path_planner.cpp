#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Path.h"
#include "path_planning/input_params.h"
#include "dynamic_reconfigure/server.h"
#include "path_planning/dyn_paramsConfig.h"

#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/Path.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/config.h"
#include <sstream>
#include <iostream>
#include <utility>

#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/samplers/UniformValidStateSampler.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/ScopedState.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <boost/math/constants/constants.hpp>
#include <time.h>
#include <stdlib.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;
ros::Publisher path_pub;    //Publisher for path
ros::Publisher obstacles_pub;   //Publisher for obstacles
std::vector<std::pair<double, std::pair<double, double>>> obstacles;    //Vector of obstacles

// Dynamic Reconfiguration Parameters
double dyn_range, dyn_threshold;
double computation_time;
double number_of_obstacles;
double radius_of_obstacles;
double randomize_obstacles;


//Check whether the found state is valid
class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr &si) : ob::StateValidityChecker(si) {}

    bool isValid(const ob::State *state) const
    {
        return this->clearance(state);
    }

    double clearance(const ob::State *state) const
    {
        const ob::RealVectorStateSpace::StateType *state2D =
            state->as<ob::RealVectorStateSpace::StateType>();

        double x = state2D->values[0];
        double y = state2D->values[1];

        //Check if the found state overlap with any obstacle
        for (std::vector<std::pair<double, std::pair<double, double>>>::iterator it = obstacles.begin(); it != obstacles.end(); it++)
            if (sqrt((x - (*it).second.first) * (x - (*it).second.first) + (y - (*it).second.second) * (y - (*it).second.second)) - (*it).first < 0.0)
                return false;
        return true;
    }
};


//Optimization Objective for Path Length (Tries to find better solution until elapsed time > computation time)
ob::OptimizationObjectivePtr getObjective(const ob::SpaceInformationPtr &si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

// Optimization Objective for Path Cost (Tries to find a solution does not exceed cost threshold)
ob::OptimizationObjectivePtr getThreshold(const ob::SpaceInformationPtr &si, double threshold)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ob::Cost(threshold));
    return obj;
}


//Problem Definition
std::pair<ob::ProblemDefinitionPtr, ob::SpaceInformationPtr> construct(double start_x, double start_y, double goal_x, double goal_y, double threshold)
{
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));
    space->as<ob::RealVectorStateSpace>()->setBounds(0.0, 100.0);   //Boundary Set
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));  //Check if the state is valid
    si->setup();    //Setup


    //Setting start-end points (Dynamic Parameters)
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_x;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_y;

    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_x;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_y;

    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->setStartAndGoalStates(start, goal);

    //Uncomment the following line if you want "Optimization Objective for Path Length" and comment the next line of it.
    //pdef->setOptimizationObjective(getObjective(si));
    pdef->setOptimizationObjective(getThreshold(si, threshold));
    return std::make_pair(pdef, si);
}



//Problem Solving Function (Callback of the ROS service /input_params)
bool execute(path_planning::input_params::Request &req,
             path_planning::input_params::Response &res)
{
    auto pdef = construct(req.start_x, req.start_y, req.end_x, req.end_y, dyn_threshold);       //Construction of the problem
    og::RRTstar *rrt = new og::RRTstar(pdef.second);    //Initialization of the solver algorithm
    rrt->setRange(dyn_range);   //setting RRT* range (Dynamic Parameter)

    //Creating a Planner
    ob::PlannerPtr optimizingPlanner(rrt);
    optimizingPlanner->setProblemDefinition(pdef.first);
    optimizingPlanner->setup();

    double initial_time = ros::Time::now().toSec();     //Noting the time
    ob::PlannerStatus solved = optimizingPlanner->solve(computation_time);      //Solving the problem
    res.elapsedTime = ros::Time::now().toSec() - initial_time;      //Finding the elapsed time
    
    //If solution found
    if (solved)
    {
        std::cout
            << optimizingPlanner->getName()
            << " found a solution of length "
            << pdef.first->getSolutionPath()->length()
            << " with an optimization objective value of "
            << pdef.first->getSolutionPath()->cost(pdef.first->getOptimizationObjective()) << std::endl
            << pdef.first->getSolutionCount() << std::endl;

        pdef.first->getSolutionPath().get()->print(std::cout);  //Printing solution path.
        og::PathGeometric path(dynamic_cast<const og::PathGeometric &>(*pdef.first->getSolutionPath()));    //Getting the path
        const std::vector<ob::State *> &states = path.getStates();      //Getting states
        ob::State *state;

        //Converting it to ROS Path message
        nav_msgs::Path nav_path;
        nav_path.header.frame_id = "map";
        geometry_msgs::PoseStamped pose;
        for (size_t i = 0; i < states.size(); ++i)
        {
            state = states[i]->as<ob::State>();
            double x, y;
            pose.pose.position.x = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
            pose.pose.position.y = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
            pose.header.frame_id = "base_link";
            nav_path.poses.push_back(pose);
        }


        //Check whether the solution cost is larger than cost threshold
        if (pdef.first->getSolutionPath()->length() > dyn_threshold || res.elapsedTime > computation_time)
        {
            res.isFound = false;
            res.elapsedTime = 0.0;
            return true;
        }

        //else
        path_pub.publish(nav_path);
        res.isFound = true;
        return true;
    }

    //If no solution found
    std::cout << "No solution found." << std::endl;
    res.elapsedTime = 0.0;
    res.isFound = false;
    return true;
}


//Construction of obstacles (Dynamic Parameters)
void obstacleConstruct(double N, double R, bool randomness)
{
    obstacles.clear();  //Reset the vector
    std::pair<double, std::pair<double, double>> tmp;   //Creating a temporary vector member
    srand(time(NULL));  //Use time for rand()
    visualization_msgs::MarkerArray obstacles_marker_array; //Creating marker arrays
    for (int i = 0; i < N; i++)
    {
        visualization_msgs::Marker obstacles_marker;    //Creating a marker
        
        //Obstacle Construction
        tmp.second.first = rand() % 100 - 50;
        tmp.second.second = rand() % 100 - 50;
        tmp.first = R;
        if (randomness)
            tmp.first = rand() % (int)R + 0.5;


        //Marker Construction
        obstacles_marker.header.frame_id = "map";
        obstacles_marker.header.stamp = ros::Time();
        obstacles_marker.ns = "obstacle";
        obstacles_marker.id = i;
        obstacles_marker.type = visualization_msgs::Marker::SPHERE;
        obstacles_marker.action = visualization_msgs::Marker::ADD;
        obstacles_marker.pose.position.x = tmp.second.first;
        obstacles_marker.pose.position.y = tmp.second.second;
        obstacles_marker.pose.position.z = 0;
        obstacles_marker.pose.orientation.x = 0.0;
        obstacles_marker.pose.orientation.y = 0.0;
        obstacles_marker.pose.orientation.z = 0.0;
        obstacles_marker.pose.orientation.w = 1.0;
        obstacles_marker.scale.x = tmp.first/2;
        obstacles_marker.scale.y = tmp.first/2;
        obstacles_marker.scale.z = 0.1;
        obstacles_marker.color.a = 1.0;
        obstacles_marker.color.r = 0.0;
        obstacles_marker.color.g = 1.0;
        obstacles_marker.color.b = 0.0;

        //Appending the created obstacle
        obstacles.push_back(tmp);
        obstacles_marker_array.markers.push_back(obstacles_marker);
    }
    obstacles_pub.publish(obstacles_marker_array);  //Publishing the obstacles as MarkerArray
}


//Dynamic Parameters Callback
void callback(path_planning::dyn_paramsConfig &config, uint32_t level)
{
    dyn_range = config.RRTstar_Range, dyn_threshold = config.Cost_Threshold;
    computation_time = config.Max_Solution_Time;
    number_of_obstacles = config.Number_of_Obstacles;
    radius_of_obstacles = config.Radius_of_Obstacles;
    randomize_obstacles = config.Randomize_Obstacle_Radius;
    obstacleConstruct(number_of_obstacles, radius_of_obstacles, randomize_obstacles);
}

//Main Function
int main(int argc, char **argv)
{
    //Initializing ROS Node, Publishers and Service
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;
    path_pub = nh.advertise<nav_msgs::Path>("Path_Publisher", 1000);
    obstacles_pub = nh.advertise<visualization_msgs::MarkerArray>("Obstacles_Publisher", 1000);
    ros::ServiceServer service = nh.advertiseService("input_params", execute);
    
    //Initializing Dynamic Reconfigure
    dynamic_reconfigure::Server<path_planning::dyn_paramsConfig> server;
    dynamic_reconfigure::Server<path_planning::dyn_paramsConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ROS_INFO("path_planner node has been started.");
    ros::spin();

    return 0;
}
