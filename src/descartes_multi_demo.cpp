#include "hydra_demos/descartes_multi_demo.h"

namespace hydra_demos
{
DescartesMultiDemo::DescartesMultiDemo(const ros::NodeHandle& nh, const PlannerConfigs& configs)
    : configs_(configs)
{
    moveit_cpp_ptr_.reset(new moveit_cpp::MoveItCpp(nh));  
    moveit_cpp_ptr_->getPlanningSceneMonitor()->providePlanningSceneService();

    // create planning components
    for (const auto& config : configs_)
    {
        planning_components_.emplace_back(new moveit_cpp::PlanningComponent(config.group_name, moveit_cpp_ptr_));
    }
}

void DescartesMultiDemo::moveHome()
{
    // plan in series (for now)
    for (const auto& planning_component : planning_components_)
    {
        planning_component->setStartStateToCurrentState();
        planning_component->setGoal(HOME_POSITION_NAME);
    
        planning_component->plan();
        planning_component->execute(false);
    }  
}

} // namespace hydra_demos