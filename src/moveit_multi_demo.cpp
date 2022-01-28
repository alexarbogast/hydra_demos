#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <eigen_conversions/eigen_msg.h>
#include "gcode_core/core/gcode.h"
#include "gcode_core/flavor_impl/marlin_gcode.h"

const std::string HOME_POSITION_NAME = "home";

using namespace gcode_core;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hydra_moveit_multi_demo");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // open gcode file to extract path
    std::string filepath = "/home/alex/Documents/parts/cube_bottom.gcode";

    GcodeBase gcode;
    Marlin::ParseGcode(filepath, gcode);

    // setup planning interface
    static const std::string PLANNING_GROUP = "hydra_planning_group";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    move_group_interface.setMaxVelocityScalingFactor(0.25);
    move_group_interface.setPoseReferenceFrame("world");

    // print basic information
    ROS_INFO("Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

    // move home
    std::map<std::string, double> home_joint_values = move_group_interface.getNamedTargetValues("home");
    move_group_interface.setNamedTarget(HOME_POSITION_NAME);
    move_group_interface.move();

    // move to first coordinate
    //geometry_msgs::Pose target_pose;
    //Eigen::Isometry3d test(gcode.toolpath().front().front().front());
    //test.translation() /= 1000;
    //test.linear() = Eigen::Quaterniond(0, 0, 1, 0).matrix();
//
    //ros::Duration(10).sleep();
//
    //tf::poseEigenToMsg(test, target_pose);
    //move_group_interface.setStartStateToCurrentState();
//
    //move_group_interface.setPoseTarget(target_pose, "rob1_typhoon_extruder");    
    //
    //moveit::planning_interface::MoveGroupInterface::Plan start_plan;
//
    //if (move_group_interface.plan(start_plan))
    //{
    //    ROS_INFO("moving to initial pose");
    //    move_group_interface.execute(start_plan);
    //}
    //else

    // move through first bead
    //for (const auto& bead : gcode.toolpath().front())
    //{
    //    std::vector<geometry_msgs::Pose> waypoints;
    //    for (const auto& cmd : *bead)
    //    {
    //        geometry_msgs::Pose target_pose;
    //        Eigen::Isometry3d test(cmd->getWaypoint());
    //        test.translation() /= 1000;
    //        test.linear() = Eigen::Quaterniond(0, 0, 1, 0).matrix();
//
    //        tf::poseEigenToMsg(test, target_pose);
    //        waypoints.push_back(target_pose);
    //    }
//
    //    moveit_msgs::RobotTrajectory trajectory;
    //    const double jump_threshold = 0.0;
    //    const double eef_step = 0.01;
    //    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
//
    //    move_group_interface.execute(trajectory);
    //}    

    // move back home
    move_group_interface.setJointValueTarget(home_joint_values);
    move_group_interface.move();

    return 0;
}