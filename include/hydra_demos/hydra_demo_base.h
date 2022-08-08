#ifndef HYDRA_DEMO_BASE
#define HYDRA_DEMO_BASE

#include <moveit/move_group_interface/move_group_interface.h>
#include "mrmf_core/multi_robot_group.h"

static const std::string PLANNING_GROUP = "hydra_planning_group";
static const std::string HOME_POSITION = "home";

using namespace mrmf_core;

namespace hydra_demos
{
class HydraDemo
{
public:
    HydraDemo()
        : move_group_interface_(PLANNING_GROUP),
          multi_robot_group_(PLANNING_GROUP, move_group_interface_.getRobotModel())
    {
        robot1 = multi_robot_group_.addRobot("rob1_planning_group", "rob1_typhoon_extruder", "rob1_base", Robot::RobotType::MANIPULATOR);
        robot2 = multi_robot_group_.addRobot("rob2_planning_group", "rob2_typhoon_extruder", "rob2_base", Robot::RobotType::MANIPULATOR);
        robot3 = multi_robot_group_.addRobot("rob3_planning_group", "rob3_typhoon_extruder", "rob3_base", Robot::RobotType::MANIPULATOR);
        positioner = multi_robot_group_.addRobot("positioner_planning_group", "positioner", Robot::RobotType::POSITIONER);
    }

    void moveHome()
    {
        move_group_interface_.setNamedTarget(HOME_POSITION);
        move_group_interface_.move();
    }

protected:
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    mrmf_core::MultiRobotGroup multi_robot_group_;

    mrmf_core::RobotID robot1, robot2, robot3, positioner;
};

} // namespace hydra_demos 

#endif // HYDRA_DEMO_BASE