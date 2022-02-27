#ifndef BIO_IK_DEMO_H
#define BIO_IK_DEMO_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include "bio_ik/bio_ik.h"

const std::string PLANNING_GROUP = "hydra_planning_group";
const std::string HOME_POSITION = "home";

namespace bio_ik
{
class RelativePositionGoal : public PositionGoal
{
    std::string base_frame_;

public:
    RelativePositionGoal()
        : PositionGoal()
    { 
    }

    RelativePositionGoal(const std::string& link_name, const tf2::Vector3& position, const std::string& frame, double weight = 1.0)
        : PositionGoal(link_name, position, weight)
    {
    }

    inline const std::string& getBaseFrame() const { return base_frame_; }
    inline void setBaseFrame(const std::string& frame) { base_frame_ = frame; }

    virtual void describe(GoalContext& context) const override
    {
        Goal::describe(context);
        
        context.addLink(getLinkName());
        context.addLink(base_frame_);
    }

    virtual double evaluate(const GoalContext& context) const override
    {
        Frame base_frame = context.getLinkFrame(1);
        Frame target(getPosition(), tf2::Quaternion());

        tf2::Vector3 goal_position = (base_frame * target).getPosition();

        return context.getLinkFrame().getPosition().distance2(goal_position);
    } 
};

} // namespace bio_ik

namespace hydra_demos
{
struct CartesianPath
{
    EigenSTL::vector_Vector3d path_;
    std::string tip_link_;
    std::string base_frame_;

    inline size_t size() const { return path_.size(); } 
    inline Eigen::Vector3d& operator[] (size_t pos) { return path_[pos]; }
    inline const Eigen::Vector3d& operator[] (size_t pos) const { return path_[pos]; }
};

typedef std::vector<CartesianPath> MultiRobotPath;

class BioIKDemo
{
public:
    BioIKDemo();

    void moveHome();

    void setDefaultIKOptions(bio_ik::BioIKKinematicsQueryOptions& options) const;
    void createRobotTrajectory();
    void planMultiRobotTrajectory(const MultiRobotPath& paths);
    void execute();
    void test();
private:
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    robot_trajectory::RobotTrajectoryPtr robot_trajectory_;
};

} // namespace hydra_demos

#endif // BIO_IK_DEMO_H
