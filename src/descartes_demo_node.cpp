#include <ros/ros.h>

#include "hydra_demos/descartes_demo.h"
#include "gcode_core/flavor_impl/marlin_gcode.h"

using namespace hydra_demos;


int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "descartes_demo");
    ros::NodeHandle nh("/descartes_demo");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    /* Otherwise robot with zeros joint_states */
    ros::Duration(1.0).sleep();

    // configuration
    CartesianPlannerConfig config;
    config.group_name = "rob1_planning_group";
    config.world_frame = "positioner_static";
    config.base_link = "rob1_base_link";
    config.tip_link = "rob1_typhoon_extruder";

    DescartesDemo application(nh, config);

    // open gcode file to extract path
    std::string filepath = "/home/alex/Documents/parts/cube_bottom.gcode";

    gcode_core::GcodeBase gcode;
    gcode_core::Marlin::ParseGcode(filepath, gcode);

    // execute trajectory
    application.moveHome();

    //move through first bead
    for (const auto& bead : gcode.toolpath().front())
    {
        EigenSTL::vector_Isometry3d waypoints;
        for (auto& cmd : *bead)
        {
            Eigen::Isometry3d target_pose(cmd->getWaypoint());
            target_pose.translation() /= 1000;
            target_pose.linear() = Eigen::Quaterniond(0, 0, 1, 0).matrix();

            waypoints.push_back(target_pose);
        }
        
        const double eef_step = 0.005;
        application.planAndExecuteCartesianPath(waypoints, eef_step);
    }  

    application.moveHome();
    spinner.stop();

    return 0;
}