#ifndef DESCARTES_MULTI_DEMO_H
#define DESCARTES_MULTI_DEMO_H

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include "hydra_demos/cartesian_planner.h"

namespace hydra_demos
{
typedef std::vector<CartesianPlannerConfig> PlannerConfigs;

struct MultiDemoConfig
{   

    PlannerConfigs planner_configs;
};

class DescartesMultiDemo
{
public:
    typedef std::vector<moveit_cpp::PlanningComponentPtr> PlanningComponents;

    DescartesMultiDemo(const ros::NodeHandle& nh, const PlannerConfigs& configs);

    void setStateFromIKSubgroups(robot_state::RobotState& robot_state,
                                 const robot_model::JointModelGroup* joint_model_group,
                                 const EigenSTL::vector_Isometry3d& poses,
                                 const ros::V_string& tips);
    void moveHome();
    void MultiMoveToPose(const EigenSTL::vector_Isometry3d poses);

protected:
    PlannerConfigs configs_;

    moveit_cpp::MoveItCppPtr moveit_cpp_ptr_;
    moveit_cpp::PlanningComponentPtr planning_component_; 
}; 

} // namespace hydra_demos

#endif // DESCARTES_MULTI_DEMO_H