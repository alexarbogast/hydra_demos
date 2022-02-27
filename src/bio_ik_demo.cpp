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
    robot_trajectory_.reset(new RobotTrajectory(move_group_interface_.getRobotModel()));
}

void BioIKDemo::moveHome()
{
    move_group_interface_.setNamedTarget(HOME_POSITION);
    move_group_interface_.move();
}

void BioIKDemo::planMultiRobotTrajectory(const MultiRobotPath& paths)
{
    moveit::core::RobotModelConstPtr robot_model = move_group_interface_.getRobotModel();
    const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(PLANNING_GROUP);

    robot_state::RobotState robot_state(robot_model);

    // reset robot trajectory
    robot_trajectory_.reset(new RobotTrajectory(robot_model));

    static double delta_time = 1;
    robot_trajectory_->addSuffixWayPoint(robot_state, delta_time);

    // remember initial positions
    size_t max_length = 0;
    for (const CartesianPath& path : paths)
    {
        Eigen::Isometry3d init_position = robot_state.getFrameTransform(path.tip_link_);

        size_t path_length = path.size();
        max_length = path_length > max_length ? path_length : max_length;
    }
    
    robot_state::RobotState robot_state_ik(robot_model);
    for (size_t i = 0; i < max_length; i++)
    {
        bio_ik::BioIKKinematicsQueryOptions ik_options;
        setDefaultIKOptions(ik_options);

        for (const CartesianPath& path : paths)
        {
            if (i < path.size())
            {
                auto* position_goal = new bio_ik::RelativePositionGoal();
                position_goal->setBaseFrame(path.base_frame_);
                position_goal->setLinkName(path.tip_link_);

                Eigen::Vector3d p = path[i];
                position_goal->setPosition(tf2::Vector3(p.x(), p.y(), p.z()));

                auto* direction_goal = new bio_ik::DirectionGoal();
                direction_goal->setAxis(tf2::Vector3(0, 0, 1));
                direction_goal->setDirection(tf2::Vector3(0, 0, -1));
                direction_goal->setLinkName(path.tip_link_);

                ik_options.goals.emplace_back(position_goal);
                ik_options.goals.emplace_back(direction_goal);
            }
        }

        bool success = robot_state_ik.setFromIK(
            joint_model_group,              // joints to be used for IK
            EigenSTL::vector_Isometry3d(),  // empty end_effector positions
            std::vector<std::string>(),     // empty tip link names
            0.1,                            // solver timeout
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

    //trajectory_processing::IterativeParabolicTimeParameterization time_param;
    //time_param.computeTimeStamps(*robot_trajectory_);
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

void BioIKDemo::execute()
{
    moveit_msgs::RobotTrajectory trajectory_msg;
    robot_trajectory_->getRobotTrajectoryMsg(trajectory_msg);

    std::cout << trajectory_msg << std::endl;

    move_group_interface_.setStartStateToCurrentState();
    move_group_interface_.execute(trajectory_msg);
}


void BioIKDemo::test()
{
    // create planning constraints
    bio_ik::BioIKKinematicsQueryOptions ik_options;
    ik_options.replace = true;
    ik_options.return_approximate_solution = true;

    // position goal constraints
    auto* rob1_position_goal = new bio_ik::RelativePositionGoal();
    rob1_position_goal->setBaseFrame("positioner");
    rob1_position_goal->setLinkName("rob1_typhoon_extruder");
    rob1_position_goal->setPosition(tf2::Vector3(-0.2, -0.2, 0.00));

    auto* rob2_position_goal = new bio_ik::RelativePositionGoal();
    rob2_position_goal->setBaseFrame("positioner");
    rob2_position_goal->setLinkName("rob2_typhoon_extruder");
    rob2_position_goal->setPosition(tf2::Vector3(0.2, -0.2, 0.00));

    auto* rob3_position_goal = new bio_ik::RelativePositionGoal();
    rob3_position_goal->setBaseFrame("positioner");
    rob3_position_goal->setLinkName("rob3_typhoon_extruder");
    rob3_position_goal->setPosition(tf2::Vector3(-0.2, 0.2, 0.00));

    // aiming goal contraints
    auto* rob1_aiming_goal = new bio_ik::DirectionGoal();
    rob1_aiming_goal->setAxis(tf2::Vector3(0, 0, 1));
    rob1_aiming_goal->setDirection(tf2::Vector3(0, 0, -1));
    rob1_aiming_goal->setLinkName("rob1_typhoon_extruder");

    auto* rob2_aiming_goal = new bio_ik::DirectionGoal();
    rob2_aiming_goal->setAxis(tf2::Vector3(0, 0, 1));
    rob2_aiming_goal->setDirection(tf2::Vector3(0, 0, -1));
    rob2_aiming_goal->setLinkName("rob2_typhoon_extruder");

    auto* rob3_aiming_goal = new bio_ik::DirectionGoal();
    rob3_aiming_goal->setAxis(tf2::Vector3(0, 0, 1));
    rob3_aiming_goal->setDirection(tf2::Vector3(0, 0, -1));
    rob3_aiming_goal->setLinkName("rob3_typhoon_extruder");

    auto* use_pos_joint = new bio_ik::LinkGoalBase();
    use_pos_joint->setLinkName("positioner");

    // minimal displacement constraint
    auto* minimal_displacement = new bio_ik::MinimalDisplacementGoal();

    ik_options.goals.emplace_back(rob1_position_goal);
    ik_options.goals.emplace_back(rob1_aiming_goal);
    ik_options.goals.emplace_back(rob2_position_goal);
    ik_options.goals.emplace_back(rob2_aiming_goal);
    ik_options.goals.emplace_back(rob3_position_goal);
    ik_options.goals.emplace_back(rob3_aiming_goal);
    ik_options.goals.emplace_back(use_pos_joint);
    ik_options.goals.emplace_back(minimal_displacement);

    moveit::core::RobotModelConstPtr robot_model = move_group_interface_.getRobotModel();
    const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(PLANNING_GROUP);

    robot_state::RobotState robot_state_ik(robot_model);
    bool success = robot_state_ik.setFromIK(
        joint_model_group,              // joints to be used for IK
        EigenSTL::vector_Isometry3d(),  // empty end_effector positions
        std::vector<std::string>(),     // empty tip link names
        0.1,                            // solver timeout
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

    // move to contrained target
    move_group_interface_.setStartStateToCurrentState();
    move_group_interface_.setJointValueTarget(robot_state_ik);
    move_group_interface_.move();
}

} //  namespace hydra_demos