#include "hydra_demos/descartes_multi_demo.h"
#include "gcode_core/flavor_impl/marlin_gcode.h"

using namespace hydra_demos;


int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "descartes_demo");
    ros::NodeHandle nh("/descartes_demo");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // configurations
    CartesianPlannerConfig rob1_config, rob2_config, rob3_config;
    rob1_config.group_name = "rob1_planning_group";
    rob1_config.world_frame = "positioner_static";
    rob1_config.base_link = "rob1_base_link";
    rob1_config.tip_link = "rob1_typhoon_extruder";

    rob2_config.group_name = "rob2_planning_group";
    rob2_config.world_frame = "positioner_static";
    rob2_config.base_link = "rob2_base_link";
    rob2_config.tip_link = "rob2_typhoon_extruder";

    rob3_config.group_name = "rob3_planning_group";
    rob3_config.world_frame = "positioner_static";
    rob3_config.base_link = "rob3_base_link";
    rob3_config.tip_link = "rob3_typhoon_extruder";

    DescartesMultiDemo::PlannerConfigs planner_configs;
    planner_configs.push_back(rob1_config);
    planner_configs.push_back(rob2_config);  
    planner_configs.push_back(rob3_config);  

    DescartesMultiDemo application(nh, planner_configs);

    // open gcode file to extract path
    std::string filepath = "/home/alex/Documents/parts/cube_bottom.gcode";

    gcode_core::GcodeBase gcode;
    gcode_core::Marlin::ParseGcode(filepath, gcode);

    ros::Duration(15.0).sleep();
    // execute trajectory
    application.moveHome();

    ros::waitForShutdown();
    spinner.stop();

    return 0;
}