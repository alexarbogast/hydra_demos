#ifndef CARTESIAN_PLANNER_H
#define CARTESIAN_PLANNER_H

#include <descartes_moveit/ikfast_moveit_state_adapter.h>
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_planner/dense_planner.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string HOME_POSITION_NAME = "home";

typedef std::vector<descartes_core::TrajectoryPtPtr> DescartesTrajectory;

namespace hydra_demos
{
struct CartesianPlannerConfig
{
    std::string group_name;
    std::string world_frame;
    std::string base_link;
    std::string tip_link;
};

class CartesianPlanner
{
public:
    CartesianPlanner() = default;

    void initialize(const CartesianPlannerConfig& config, 
                    const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

    descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Isometry3d& pose);

    void computeCartesianPath(const EigenSTL::vector_Isometry3d& waypoints, double eef_step, 
                              robot_trajectory::RobotTrajectory& joint_trajectory);

    void createDensePath(const EigenSTL::vector_Isometry3d& waypoints, double eef_step,
                                           DescartesTrajectory& trajectory);

    void planCartesianPath(DescartesTrajectory& input_traj, DescartesTrajectory& output_traj);
    void planCartesianPath(DescartesTrajectory& input_traj, robot_trajectory::RobotTrajectory& output_traj);

    void copyDescartesResultToRobotTrajectory(const DescartesTrajectory descartes_result,
                                              robot_trajectory::RobotTrajectory& robot_trajectory);
    void timeParameterizeTrajectory(robot_trajectory::RobotTrajectory& output_traj);

private:
    CartesianPlannerConfig config_;

    descartes_core::RobotModelPtr robot_model_ptr_;
    descartes_planner::DensePlanner planner_;

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
};

} // namepspace hydra_demos

#endif // CARTESIAN_PLANNER_H