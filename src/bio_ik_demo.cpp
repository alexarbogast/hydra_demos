#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include "hydra_demos/bio_ik_demo.h"

using namespace robot_trajectory;

namespace hydra_demos
{
BioIKDemo::BioIKDemo()
    : move_group_interface_(PLANNING_GROUP)
{
    move_group_interface_.setMaxVelocityScalingFactor(0.2);
}

void BioIKDemo::moveHome()
{
    move_group_interface_.setNamedTarget(HOME_POSITION);
    move_group_interface_.move();
}

void BioIKDemo::planMultiRobotTrajectory(MultiRobotPath& paths)
{
    moveit::core::RobotModelConstPtr robot_model = move_group_interface_.getRobotModel();
    const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(PLANNING_GROUP);

    //robot_state.setToDefaultValues();
    robot_state::RobotState robot_state(*(move_group_interface_.getCurrentState()));

    // reset robot trajectory
    robot_trajectory_.reset(new RobotTrajectory(robot_model, PLANNING_GROUP));

    static double delta_time = 0.05;
    robot_trajectory_->addSuffixWayPoint(robot_state, delta_time);

    // interpolate dense path
    MultiRobotPath dense_paths;
    createDensePaths(paths, dense_paths, 0.01);

    // remember initial positions
    size_t max_length = 0;
    for (const CartesianPath& path : dense_paths)
    {
        size_t path_length = path.size();
        max_length = path_length > max_length ? path_length : max_length;
    }
    
    robot_state::RobotState robot_state_ik(robot_state);
    for (size_t i = 0; i < max_length; i++)
    {
        bio_ik::BioIKKinematicsQueryOptions ik_options;
        setDefaultIKOptions(ik_options);

        for (const CartesianPath& path : dense_paths)
        {
            if (i < path.size())
            {
                auto* position_goal = new bio_ik::RelativePositionGoal();
                position_goal->setBaseFrame(path.base_frame);
                position_goal->setLinkName(path.tip_link);

                Eigen::Vector3d p = path[i];
                position_goal->setPosition(tf2::Vector3(p.x(), p.y(), p.z()));

                auto* direction_goal = new bio_ik::DirectionGoal();
                direction_goal->setAxis(tf2::Vector3(0, 0, 1));
                direction_goal->setDirection(tf2::Vector3(0, 0, -1));
                direction_goal->setLinkName(path.tip_link);

                ik_options.goals.emplace_back(position_goal);
                ik_options.goals.emplace_back(direction_goal);
            }
        }

        bool success = robot_state_ik.setFromIK(
            joint_model_group,              // joints to be used for IK
            EigenSTL::vector_Isometry3d(),  // empty end_effector positions
            std::vector<std::string>(),     // empty tip link names
            0.005,                            // solver timeout
            moveit::core::GroupStateValidityCallbackFn(),
            ik_options                      // ik constraints
        );

        if (success)
        {
            ROS_INFO_STREAM("IK succeeded");
        }
        else
        {
            ROS_ERROR_STREAM("Could not set robot state from IK");
            exit(-1);
        }

        // add robot state to trajectory
        robot_trajectory_->addSuffixWayPoint(robot_state_ik, delta_time);
    }

    trajectory_processing::IterativeParabolicTimeParameterization time_param;
    time_param.computeTimeStamps(*robot_trajectory_);
}

void BioIKDemo::setDefaultIKOptions(bio_ik::BioIKKinematicsQueryOptions& options) const
{
    options.replace = true;
    options.return_approximate_solution = true;

    auto* ext_axis_enable = new bio_ik::LinkGoalBase();
    ext_axis_enable->setLinkName("positioner");
    options.goals.emplace_back(ext_axis_enable);
    
    auto* minimal_displacement = new bio_ik::MinimalDisplacementGoal();
    options.goals.emplace_back(minimal_displacement);
}

void BioIKDemo::createDensePaths(const MultiRobotPath& path_in, MultiRobotPath& path_out, double eef_step) const
{
    if (eef_step <= 0.0)
    {
        ROS_ERROR("Invaled max eef step passed into computeCartesianPath"
                    "eef step must be greater than zero");
        exit(-1);
    }

    // add interpolated poses to path_out
    for (const CartesianPath& path : path_in)
    {
        CartesianPath interp_path;
        interp_path.tip_link   = path.tip_link;
        interp_path.base_frame = path.base_frame;

        for (auto it = path.begin(); it != --path.end(); ++it)
        {
            Eigen::Vector3d start_pose = *it;
            Eigen::Vector3d end_pose = *std::next(it);
            
            double translation_distance = (end_pose - start_pose).norm();
            unsigned int translation_steps = floor(translation_distance / eef_step);
            translation_steps = translation_steps < 2 ? 2 : translation_steps;

            for (unsigned int i = 0; i <= translation_steps; i++)
            {
                double percentage = (double)i / double(translation_steps);             
                interp_path.path.emplace_back(Eigen::Vector3d(percentage * end_pose + (1 - percentage) * start_pose));                
            }
        }

        path_out.emplace_back(interp_path);
    }
}

void BioIKDemo::execute()
{
    moveit_msgs::RobotTrajectory trajectory_msg;
    robot_trajectory_->getRobotTrajectoryMsg(trajectory_msg);

    move_group_interface_.setStartStateToCurrentState();
    move_group_interface_.execute(trajectory_msg);
}

} //  namespace hydra_demos