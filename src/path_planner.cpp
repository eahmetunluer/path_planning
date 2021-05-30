#include "ros/ros.h"
#include "std_msgs/String.h"
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
namespace ob = ompl::base;
namespace og = ompl::geometric;

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

std::pair<ob::ProblemDefinitionPtr, ob::SpaceInformationPtr> construct(double start_x, double start_y, double goal_x, double goal_y)
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
    pdef->setOptimizationObjective(getObjective(si));
    return std::make_pair(pdef, si);
}

int main(int argc, char **argv)
{
    double computation_time = 4.0;
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;

    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);

    int count = 0;
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    std::cout << std::endl
              << std::endl;

    auto pdef = construct(0.0, 0.0, 50.0, 50.0);
    og::RRTstar* rrt = new og::RRTstar(pdef.second);
    rrt->setRange(1.0);

    ob::PlannerPtr optimizingPlanner(rrt);

    optimizingPlanner->setProblemDefinition(pdef.first);
    optimizingPlanner->setup();
    ob::PlannerStatus solved = optimizingPlanner->solve(computation_time);
    if (solved)
    {
        // Output the length of the path found
        std::cout
            << optimizingPlanner->getName()
            << " found a solution of length "
            << pdef.first->getSolutionPath()->length()
            << " with an optimization objective value of "
            << pdef.first->getSolutionPath()->cost(pdef.first->getOptimizationObjective()) << std::endl;
        // If a filename was specified, output the path as a matrix to
        // that file for visualization
        //if (!outputFile.empty())
        //{
        //    std::ofstream outFile(outputFile.c_str());
        //    std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->printAsMatrix(outFile);
        //    outFile.close();
        //}
    }
    else
        std::cout << "No solution found." << std::endl;

    return 0;
}
