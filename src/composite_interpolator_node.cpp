#include <hydra_demos/hydra_demo_base.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <chrono>

#include <mrmf_core/composite_interpolator.h>

static const std::string ROB1_BASE_NAME = "rob1_base_link";
static const std::string ROB2_BASE_NAME = "rob2_base_link";
static const std::string ROB3_BASE_NAME = "rob3_base_link";

static const std::string ROB1_TIP_NAME = "rob1_typhoon_extruder";
static const std::string ROB2_TIP_NAME = "rob2_typhoon_extruder";
static const std::string ROB3_TIP_NAME = "rob3_typhoon_extruder";

static const std::string POS_LINK_NAME = "positioner";

using namespace mrmf_core;

namespace hydra_demos
{
class CompositeInterpolatorDemo : public HydraDemo
{
public:

    void run()
    {
        // get each robot's base orientation to use as the orientation for assigned poses
        robot_state::RobotStatePtr current_state = move_group_interface_.getCurrentState();

        Eigen::AngleAxisd roty = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
        Eigen::Isometry3d rob1_static_orient = current_state->getFrameTransform(ROB1_BASE_NAME) * roty;
        Eigen::Isometry3d rob2_static_orient = current_state->getFrameTransform(ROB2_BASE_NAME) * roty;
        Eigen::Isometry3d rob3_static_orient = current_state->getFrameTransform(ROB3_BASE_NAME) * roty;

        // ========================== FIRST MOVE ==========================
        Eigen::Isometry3d T_world_pos = current_state->getGlobalLinkTransform(POS_LINK_NAME);
    
        // robot positions in positioner frame
        Eigen::Isometry3d rob1_p1 = T_world_pos * Eigen::Translation3d(-0.3, -0.3, 0.0);
        Eigen::Isometry3d rob3_p1 = T_world_pos * Eigen::Translation3d(-0.3,  0.3, 0.0);

        rob1_p1.linear() = rob1_static_orient.linear();
        rob3_p1.linear() = rob3_static_orient.linear();

        robot_state::RobotState state1 = *current_state;
        state1.setFromIK(multi_robot_group_.getJointModelGroup(robot1), rob1_p1, ROB1_TIP_NAME, 0.0);
        state1.setFromIK(multi_robot_group_.getJointModelGroup(robot3), rob3_p1, ROB3_TIP_NAME, 0.0);

        // ========================== SECOND MOVE ==========================
        robot_state::RobotState state2 = state1;

        // change positioner angle
        state2.setJointGroupPositions(multi_robot_group_.getJointModelGroup(positioner), {-M_PI / 3.0});
        T_world_pos = state2.getGlobalLinkTransform(POS_LINK_NAME);

        // robot positions in positioner frame
        Eigen::Isometry3d rob1_p2 = T_world_pos * Eigen::Translation3d( 0.3, -0.3, 0.0);
        Eigen::Isometry3d rob2_p2 = T_world_pos * Eigen::Translation3d( 0.3,  0.3, 0.0);
        Eigen::Isometry3d rob3_p2 = T_world_pos * Eigen::Translation3d(-0.3,  0.3, 0.0);

        rob1_p2.linear() = rob1_static_orient.linear();
        rob2_p2.linear() = rob2_static_orient.linear();
        rob3_p2.linear() = rob3_static_orient.linear();

        state2.setFromIK(multi_robot_group_.getJointModelGroup(robot1), rob1_p2, ROB1_TIP_NAME, 0.0);
        state2.setFromIK(multi_robot_group_.getJointModelGroup(robot2), rob2_p2, ROB2_TIP_NAME, 0.0);
        state2.setFromIK(multi_robot_group_.getJointModelGroup(robot3), rob3_p2, ROB3_TIP_NAME, 0.0);

        // ====================== COMPOSITE INTERPOLATION ========================
        std::vector<RobotPtr> robots = {multi_robot_group_.getRobot(robot1),
                                        multi_robot_group_.getRobot(robot2),
                                        multi_robot_group_.getRobot(robot3),
                                        multi_robot_group_.getRobot(positioner)};

        moveit::core::MaxEEFStep max_step(0.001);

        std::vector<InterpType> types = {InterpType::LINEAR, InterpType::JOINT, InterpType::LINEAR, InterpType::JOINT};
        std::vector<int> coord_indices = {3, -1, 3, -1};
        std::vector<robot_state::RobotStatePtr> output_traj;
        bool success = CompositeInterpolator::interpolate(&state1, &state2, robots, types, coord_indices, max_step, output_traj);  

        if (success)
        {
            // add to robot traj
            robot_trajectory::RobotTrajectory traj(move_group_interface_.getRobotModel(), PLANNING_GROUP);
            for (auto& state : output_traj)
            {
                traj.addSuffixWayPoint(state, 0.01);
            }

            // convert to trajectory message
            moveit_msgs::RobotTrajectory trajectory_msg;
            traj.getRobotTrajectoryMsg(trajectory_msg);

            move_group_interface_.setJointValueTarget(state1);
            move_group_interface_.move();

            move_group_interface_.execute(trajectory_msg);
            moveHome();
        }
        else
        {
            ROS_ERROR("Composite interpolation ailed");
            exit(-1);
        } 
    }
};

} // namespace hydra_demos 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "composite_interpolator_demo_node");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    hydra_demos::CompositeInterpolatorDemo application;
    application.run();

    return 0;
}