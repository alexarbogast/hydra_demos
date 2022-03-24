#include "hydra_demos/descartes_multi_demo.h"
#include "gcode_core/flavor_impl/marlin_gcode.h"

using namespace hydra_demos;

class CompositeMoveitStateAdapter
{
public:
    CompositeMoveitStateAdapter(const moveit::core::RobotState& robot_state, const std::string& group_name)
    {
        
    }

};

class DescartesCompositePlanner
{
public:
    DescartesCompositePlanner(const ros::NodeHandle& nh)
    {
        moveit_cpp_ptr_.reset(new moveit_cpp::MoveItCpp(nh));

        descartes_robot_model_.reset(new descartes_moveit::MoveitStateAdapter());
    }

private:
    moveit_cpp::MoveItCppPtr moveit_cpp_ptr_;
    descartes_core::RobotModelPtr descartes_robot_model_;
};

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "descartes_demo");
    ros::NodeHandle nh("/descartes_demo");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // configurations
    CartesianPlannerConfig positioner_config, rob1_config, rob2_config, rob3_config;
    positioner_config.group_name = "positioner_planning_group";
    positioner_config.world_frame = "world";
    positioner_config.base_link = "rotary_table_base";
    positioner_config.tip_link = "positioner";
    
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

    PlannerConfigs planner_configs;
    planner_configs.push_back(positioner_config);
    planner_configs.push_back(rob1_config);
    planner_configs.push_back(rob2_config);  
    planner_configs.push_back(rob3_config);  

    DescartesMultiDemo application(nh, planner_configs);

    // open gcode file to extract path
    std::string filepath = "/home/alex/Documents/parts/cube_bottom.gcode";

    gcode_core::GcodeBase gcode;
    gcode_core::Marlin::ParseGcode(filepath, gcode);

    // first pose
    Eigen::Isometry3d rob1_pose1 = Eigen::Isometry3d::Identity() * Eigen::Translation3d(-0.25, -0.25, 0.3) *
                                  Eigen::Quaterniond(0, 0, 1.0, 0);

    Eigen::Isometry3d rob2_pose1 = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.1, 0, 0.3) *
                                  Eigen::Quaterniond(0, 1.0, 0, 0);

    Eigen::Isometry3d rob3_pose1 = Eigen::Isometry3d::Identity() * Eigen::Translation3d(-0.25, 0.25, 0.3) *
                                  Eigen::Quaterniond(0, 0, 1.0, 0);

    Eigen::Isometry3d positioner_pose1 = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.20955) *
                                        Eigen::Quaterniond(0.9848078, 0, 0, 0.1736482);

    // second pose
    Eigen::Isometry3d rob1_pose2 = Eigen::Isometry3d::Identity() * Eigen::Translation3d(-0.1, -0.3, 0.3) *
                                   Eigen::Quaterniond(0, 0.258819, 0.9659258, 0);

    Eigen::Isometry3d rob2_pose2 = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.2, 0.1, 0.3) *
                                  Eigen::Quaterniond(0, 1.0, 0, 0);

    Eigen::Isometry3d rob3_pose2 = Eigen::Isometry3d::Identity() * Eigen::Translation3d(-0.2, 0.3, 0.3) *
                                  Eigen::Quaterniond(0, 0, 1.0, 0);

    Eigen::Isometry3d positioner_pose2 = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.20955) *
                                        Eigen::Quaterniond(0.8660254 , 0, 0, -0.5);
    
    EigenSTL::vector_Isometry3d pose1 = { positioner_pose1, rob1_pose1, rob2_pose1, rob3_pose1 };
    EigenSTL::vector_Isometry3d pose2 = { positioner_pose2, rob1_pose2, rob2_pose2, rob3_pose2 };
    
    // execute trajectory
    const int number_of_cycles = 5;
    for (int i = 0; i < number_of_cycles; i++)
    {
        application.moveHome();

        application.MultiMoveToPose(pose1);
        application.MultiMoveToPose(pose2);

        application.moveHome();
    }

    spinner.stop();
    ros::waitForShutdown();
    return 0;
}