#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <moveit_msgs/RobotTrajectory.h>

#include "mrmf_core/multi_robot_group.h"

static const std::string PLANNING_GROUP = "hydra_planning_group";
static const std::string HOME_POSITION = "home";

namespace hydra_demos
{
    using namespace mrmf_core;

    class MrmfDemo
{
public:
    MrmfDemo()
        : move_group_interface_(PLANNING_GROUP),
          multi_robot_group_(PLANNING_GROUP, move_group_interface_.getRobotModel())
    {
        robot1 = multi_robot_group_.addRobot("rob1_planning_group", "rob1_typhoon_extruder", Robot::RobotType::MANIPULATOR);
        robot2 = multi_robot_group_.addRobot("rob2_planning_group", "rob2_typhoon_extruder", Robot::RobotType::MANIPULATOR);
        robot3 = multi_robot_group_.addRobot("rob3_planning_group", "rob3_typhoon_extruder", Robot::RobotType::MANIPULATOR);
        positioner = multi_robot_group_.addRobot("positioner_planning_group", "positioner", Robot::RobotType::POSITIONER);
    }

    void moveHome()
    {
        move_group_interface_.setNamedTarget(HOME_POSITION);
        move_group_interface_.move();
    }

    void run()
    {
        // create waypoints of square
        Eigen::Vector3d p_a = {-0.25, -0.25, 0.25};
        Eigen::Vector3d p_b = { 0.25, -0.25, 0.0};
        Eigen::Vector3d p_c = { 0.25,  0.25, 0.25};
        Eigen::Vector3d p_d = {-0.25,  0.25, 0.0};        

        RobotTrajectoryPtr rob1_traj = RobotTrajectory::makeTrajectory(robot1);
        RobotTrajectoryPtr rob2_traj = RobotTrajectory::makeTrajectory(robot2);
        RobotTrajectoryPtr rob3_traj = RobotTrajectory::makeTrajectory(robot3);

        AxialSymmetricPt::Axis axis = AxialSymmetricPt::Axis::Z;
        Eigen::Vector3d direction(0, 0, -1);

        // this should probably be changed to unique pointers
        rob1_traj->addTrajectoryPt(std::make_shared<AxialSymmetricPt>(p_a, axis, direction, "positioner"));
        rob1_traj->addTrajectoryPt(std::make_shared<AxialSymmetricPt>(p_b, axis, direction, "positioner"));
        rob1_traj->addTrajectoryPt(std::make_shared<AxialSymmetricPt>(p_c, axis, direction, "positioner"));
        rob1_traj->addTrajectoryPt(std::make_shared<AxialSymmetricPt>(p_d, axis, direction, "positioner"));

        rob2_traj->addTrajectoryPt(std::make_shared<AxialSymmetricPt>(p_c, axis, direction, "positioner"));
        rob2_traj->addTrajectoryPt(std::make_shared<AxialSymmetricPt>(p_d, axis, direction, "positioner"));
        rob2_traj->addTrajectoryPt(std::make_shared<AxialSymmetricPt>(p_a, axis, direction, "positioner"));
        rob2_traj->addTrajectoryPt(std::make_shared<AxialSymmetricPt>(p_b, axis, direction, "positioner"));

        rob3_traj->addTrajectoryPt(std::make_shared<AxialSymmetricPt>(p_d, axis, direction, "positioner"));
        rob3_traj->addTrajectoryPt(std::make_shared<AxialSymmetricPt>(p_a, axis, direction, "positioner"));
        rob3_traj->addTrajectoryPt(std::make_shared<AxialSymmetricPt>(p_b, axis, direction, "positioner"));
        rob3_traj->addTrajectoryPt(std::make_shared<AxialSymmetricPt>(p_c, axis, direction, "positioner"));

        // create composite trajectory
        mrmf_core::CompositeTrajectory comp_traj;
        comp_traj.addTrajectory(rob1_traj);
        comp_traj.addTrajectory(rob2_traj);
        comp_traj.addTrajectory(rob3_traj);

        // plan for synchronous trajectory
        robot_trajectory::RobotTrajectory output_traj(move_group_interface_.getRobotModel(), PLANNING_GROUP);
        bool success = multi_robot_group_.planSynchronousTrajectory(comp_traj, output_traj, *(move_group_interface_.getCurrentState()));

        if (success)
        {
            // convert to trajectory message
            moveit_msgs::RobotTrajectory trajectory_msg;
            output_traj.getRobotTrajectoryMsg(trajectory_msg);

            // execute
            moveHome();    
            move_group_interface_.execute(trajectory_msg);
            moveHome();
        }
        else
        {
            ROS_ERROR("Failed to plan synchronous trajectory");
            exit(-1);
        }        
    }
    
private:
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    mrmf_core::MultiRobotGroup multi_robot_group_;

    mrmf_core::RobotID robot1, robot2, robot3, positioner;
};
} // namespace hydra_demos 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mrmf_demo_node");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    hydra_demos::MrmfDemo application;
    application.run();

    return 0;
}