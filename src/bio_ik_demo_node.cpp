#include <moveit/move_group_interface/move_group_interface.h>
#include "hydra_demos/bio_ik_demo.h"

using namespace hydra_demos;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hydra_moveit_test");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    BioIKDemo application;

    // create multi-robot path
    EigenSTL::vector_Vector3d path = {{ 0.2,  0.2, 0.0}, 
                                      { 0.2, -0.2, 0.0},
                                      {-0.2, -0.2, 0.0},
                                      {-0.2,  0.2, 0.0}};

    CartesianPath path1;
    path1.path_ = {path[2], path[3], path[0], path[1], path[2]};
    path1.base_frame_ = "positioner";
    path1.tip_link_ = "rob1_typhoon_extruder";

    CartesianPath path2;
    path2.path_ = {path[0], path[1], path[2], path[3], path[0]};
    path2.base_frame_ = "positioner";
    path2.tip_link_ = "rob2_typhoon_extruder";

    CartesianPath path3;
    path3.path_ = {path[3], path[0], path[1], path[2], path[3]};
    path3.base_frame_ = "positioner";
    path3.tip_link_ = "rob3_typhoon_extruder";

    MultiRobotPath multi_path = {path1, path2, path3};
    
    application.moveHome();

    application.planMultiRobotTrajectory(multi_path);
    application.execute();
    
    application.moveHome();    
    ros::waitForShutdown();
}