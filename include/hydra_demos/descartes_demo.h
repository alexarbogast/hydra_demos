#ifndef DESCARTES_DEMO_H
#define DESCARTES_DEMO_H

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include "hydra_demos/cartesian_planner.h"

namespace hydra_demos
{
class DescartesDemo
{
public:
    DescartesDemo(const ros::NodeHandle& nh, CartesianPlannerConfig config);

    virtual ~DescartesDemo() = default;

    void moveHome();
    void computeCartesianPath(EigenSTL::vector_Isometry3d& waypoints, double eef_step, 
                              robot_trajectory::RobotTrajectory& joint_trajectory);

    void executeTrajectory(const robot_trajectory::RobotTrajectoryPtr robot_trajectory);
    void planAndExecuteCartesianPath(EigenSTL::vector_Isometry3d& waypoints, double eef_step);

protected:
    CartesianPlannerConfig config_;
    CartesianPlanner planner_;

    //moveit::planning_interface::MoveGroupInterface move_group_interface_;
    moveit_cpp::MoveItCppPtr moveit_cpp_ptr_;
    moveit_cpp::PlanningComponentPtr planning_components_;
};

} // namespace hydra_demos

#endif // DESCARTES_DEMO_H