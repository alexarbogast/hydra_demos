#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/move_group/move_group_capability.h>

#include "hydra_demos/descartes_demo.h"

namespace hydra_demos
{

void CartesianPlanner::initialize(const CartesianPlannerConfig& config,
                                  const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
    config_ = config; 
    planning_scene_monitor_ = planning_scene_monitor;

    robot_model_ptr_.reset(new descartes_moveit::IkFastMoveitStateAdapter());

    // initialize ikfast parameter
    ros::NodeHandle nh;
    nh.setParam("ikfast_base_frame", config.base_link);
    nh.setParam("ikfast_tool_frame", config.tip_link);

    // initialize descartes
    if (!robot_model_ptr_->initialize(ROBOT_DESCRIPTION_PARAM,
                                      config.group_name,
                                      config.world_frame,
                                      config.tip_link))
    {
        ROS_ERROR("Failed to initialize robot model");
        exit(-1);
    }

    if(!planner_.initialize(robot_model_ptr_))
    {
        ROS_ERROR("Failed to initialize planned");
        exit(-1);
    }
}

descartes_core::TrajectoryPtPtr CartesianPlanner::makeTolerancedCartesianPoint(const Eigen::Isometry3d& pose)
{
    using namespace descartes_core;
    using namespace descartes_trajectory;

    return TrajectoryPtPtr(new AxialSymmetricPt(pose, 
                                                0.1f,
                                                AxialSymmetricPt::FreeAxis::Z_AXIS));
}

void CartesianPlanner::computeCartesianPath(const EigenSTL::vector_Isometry3d& waypoints, double eef_step, 
                                            robot_trajectory::RobotTrajectory& joint_trajectory)
{
    DescartesTrajectory cartesian_trajectory;
    createDensePath(waypoints, eef_step, cartesian_trajectory);

    planCartesianPath(cartesian_trajectory, joint_trajectory);
}

void CartesianPlanner::createDensePath(const EigenSTL::vector_Isometry3d& waypoints, double eef_step,
                                       DescartesTrajectory& trajectory)
{
    if (eef_step <= 0.0)
    {
        ROS_ERROR("Invaled max eef step passed into computeCartesianPath"
                    "eef step must be greater than zero");
        exit(-1);
    }

    for (auto it = waypoints.begin(); it != --waypoints.end(); ++it)
    {
        Eigen::Isometry3d start_pose = *it;
        Eigen::Isometry3d end_pose = *std::next(it);

        Eigen::Quaterniond start_quaternion(start_pose.linear());
        Eigen::Quaterniond end_quaternion(end_pose.linear());
        
        double translation_distance = (end_pose.translation() - start_pose.translation()).norm();
        unsigned int translation_steps = floor(translation_distance / eef_step);
        translation_steps = translation_steps < 2 ? 2 : translation_steps;

        for (unsigned int i = 1; i <= translation_steps; i++)
        {
            double percentage = (double)i / double(translation_steps);
            
            Eigen::Isometry3d interp_pose(start_quaternion.slerp(percentage, end_quaternion));
            interp_pose.translation() = percentage * end_pose.translation() + 
                                        (1 - percentage) * start_pose.translation();

            trajectory.emplace_back(makeTolerancedCartesianPoint(interp_pose));
        }
    }
}                        

void CartesianPlanner::planCartesianPath(DescartesTrajectory& input_traj, robot_trajectory::RobotTrajectory& output_traj)
{
    DescartesTrajectory descartes_traj;
    planCartesianPath(input_traj, descartes_traj);

    copyDescartesResultToRobotTrajectory(descartes_traj, output_traj);
    timeParameterizeTrajectory(output_traj);
}

void CartesianPlanner::planCartesianPath(DescartesTrajectory& input_traj, DescartesTrajectory& output_traj)
{  
    using namespace descartes_core;
    using namespace descartes_trajectory;

    // Modify start and end of trajectory
    std::vector<double> seed_pose{ 0.0, 0.75, 0.15, 0.0, -0.9, 0.0 };
    std::vector<double> start_pose, end_pose;
    if (input_traj.front()->getClosestJointPose(seed_pose, *robot_model_ptr_, start_pose) &&
        input_traj.back()->getClosestJointPose(seed_pose, *robot_model_ptr_, end_pose))
    {
        TrajectoryPtPtr start_joint = TrajectoryPtPtr(new JointTrajectoryPt(start_pose));
        TrajectoryPtPtr end_joint = TrajectoryPtPtr(new JointTrajectoryPt(end_pose));

        input_traj[0] = start_joint;
        input_traj[input_traj.size() - 1] = end_joint;
    }
    else
    {
        ROS_ERROR("Failed to find closest joint pose to seed");
        exit(-1);
    }

    bool succeeded = planner_.planPath(input_traj);

    if (succeeded)
    {
        ROS_INFO("Planned valid path");
    }
    else
    {
        ROS_ERROR("Failed planning a valid path");
        exit(-1);
    }

    planner_.getPath(output_traj);
}

void CartesianPlanner::copyDescartesResultToRobotTrajectory(const DescartesTrajectory descartes_result,
                                                            robot_trajectory::RobotTrajectory& robot_trajectory)
{
    std::vector<double> next_positions;

    robot_state::RobotState start_state = 
        planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_)->getCurrentState();
    
    const robot_model::JointModelGroup* jmg = start_state.getJointModelGroup(config_.group_name);

    for (const auto& result : descartes_result)
    {
        result->getNominalJointPose({}, *robot_model_ptr_, next_positions);

        start_state.setJointGroupPositions(jmg, next_positions);
    
        double delta_time = 0.00001;
        robot_trajectory.addSuffixWayPoint(start_state, delta_time);
    }
}

void CartesianPlanner::timeParameterizeTrajectory(robot_trajectory::RobotTrajectory& output_traj)
{
    trajectory_processing::IterativeParabolicTimeParameterization time_param;
    time_param.computeTimeStamps(output_traj);
}


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