#include "hydra_demos/descartes_demo.h"

namespace hydra_demos
{
DescartesDemo::DescartesDemo(const ros::NodeHandle& nh, CartesianPlannerConfig config)
    : config_(config)
{
    moveit_cpp_ptr_.reset(new moveit_cpp::MoveItCpp(nh));  
    moveit_cpp_ptr_->getPlanningSceneMonitor()->providePlanningSceneService();

    planning_components_.reset(new moveit_cpp::PlanningComponent(config_.group_name, moveit_cpp_ptr_));
    planner_.initialize(config_, moveit_cpp_ptr_->getPlanningSceneMonitor());
}

void DescartesDemo::moveHome()
{
    planning_components_->setStartStateToCurrentState();
    planning_components_->setGoal(HOME_POSITION_NAME);
    planning_components_->plan();
    planning_components_->execute();
}

void DescartesDemo::computeCartesianPath(EigenSTL::vector_Isometry3d& waypoints, double eef_step, 
                                         robot_trajectory::RobotTrajectory& joint_trajectory)
{
    planner_.computeCartesianPath(waypoints, eef_step, joint_trajectory);
}

void DescartesDemo::executeTrajectory(const robot_trajectory::RobotTrajectoryPtr robot_trajectory)
{
    // move to start
    planning_components_->setStartStateToCurrentState();
    robot_state::RobotState start_state = *(moveit_cpp_ptr_->getCurrentState());
    
    planning_components_->setGoal(robot_trajectory->getFirstWayPoint());
    planning_components_->plan();
    planning_components_->execute();

    // execute joint trajectory
    if (!moveit_cpp_ptr_->execute(config_.group_name, robot_trajectory))
    {
        ROS_ERROR("Could not execute trajectory!");
        exit(-1);
    }
}

void DescartesDemo::planAndExecuteCartesianPath(EigenSTL::vector_Isometry3d& waypoints, double eef_step)
{
    using namespace robot_trajectory;

    RobotTrajectoryPtr robot_trajectory = 
        std::make_shared<RobotTrajectory>(moveit_cpp_ptr_->getRobotModel(), config_.group_name);

    planner_.computeCartesianPath(waypoints, eef_step, *robot_trajectory);
    executeTrajectory(robot_trajectory);
}

} // namespace hydra_demos