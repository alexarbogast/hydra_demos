#ifndef DESCARTES_MULTI_DEMO_H
#define DESCARTES_MULTI_DEMO_H

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include "hydra_demos/cartesian_planner.h"

namespace hydra_demos
{
class DescartesMultiDemo
{
public:
    typedef std::vector<CartesianPlannerConfig> PlannerConfigs;
    typedef std::vector<moveit_cpp::PlanningComponentPtr> PlanningComponents;

    DescartesMultiDemo(const ros::NodeHandle& nh, const PlannerConfigs& configs);

    void moveHome();

protected:
    PlannerConfigs configs_;

    moveit_cpp::MoveItCppPtr moveit_cpp_ptr_;
    PlanningComponents planning_components_;
}; 

} // namespace hydra_demos

#endif // DESCARTES_MULTI_DEMO_H