#include "hydra_demos/descartes_multi_demo.h"

namespace hydra_demos
{
DescartesMultiDemo::DescartesMultiDemo(const ros::NodeHandle& nh, const PlannerConfigs& configs)
    : configs_(configs)
{
    moveit_cpp_ptr_.reset(new moveit_cpp::MoveItCpp(nh));  
    moveit_cpp_ptr_->getPlanningSceneMonitor()->providePlanningSceneService();

    planning_component_.reset(new moveit_cpp::PlanningComponent("hydra_planning_group", moveit_cpp_ptr_));

    // create planning components
    for (const auto& config : configs_)
    {
        
    }
}

void DescartesMultiDemo::setStateFromIKSubgroups(robot_state::RobotState& robot_state,
                                                 const robot_model::JointModelGroup* joint_model_group,
                                                 const EigenSTL::vector_Isometry3d& poses,
                                                 const ros::V_string& tips)
{
    std::vector<const robot_model::JointModelGroup*> subgroups;
    joint_model_group->getSubgroups(subgroups);

    for (std::size_t i = 0; i < subgroups.size(); i++)
    {
        if (!robot_state.setFromIK(subgroups[i], poses[i], tips[i]))
        {
            ROS_ERROR_STREAM("Failed to set subgroup '" << subgroups[i]->getName() << "' from IK");
        }
    }
}

void DescartesMultiDemo::moveHome()
{
    // move home
    planning_component_->setStartStateToCurrentState();
    planning_component_->setGoal(HOME_POSITION_NAME);
    planning_component_->plan();
    planning_component_->execute();
}

void DescartesMultiDemo::MultiMoveToPose(const EigenSTL::vector_Isometry3d poses)
{
    planning_component_->setStartStateToCurrentState();
    robot_state::RobotState start_state = 
        planning_scene_monitor::LockedPlanningSceneRO(moveit_cpp_ptr_->getPlanningSceneMonitor())->getCurrentState();

    ros::V_string tips;
    for (const auto& config : configs_)
        tips.emplace_back(config.tip_link);

    const robot_model::JointModelGroup* jmg = start_state.getJointModelGroup("hydra_planning_group");
    setStateFromIKSubgroups(start_state, jmg, poses, tips);
    
    planning_component_->setGoal(start_state);
    
    if (!planning_component_->plan())
    {
        ROS_ERROR("Failed to plan valid trajectory!");
        exit(-1);
    }

    if (!planning_component_->execute())
    {
        ROS_ERROR("Failed to execute trajectory!");
        exit(-1);
    }
}

} // namespace hydra_demos