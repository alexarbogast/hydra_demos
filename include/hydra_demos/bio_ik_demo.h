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
    EigenSTL::vector_Vector3d path;
    std::string tip_link;
    std::string base_frame;

    inline size_t size() const { return path.size(); } 
    inline Eigen::Vector3d& operator[] (size_t pos) { return path[pos]; }
    inline const Eigen::Vector3d& operator[] (size_t pos) const { return path[pos]; }

    EigenSTL::vector_Vector3d::iterator begin() { return path.begin(); }
    EigenSTL::vector_Vector3d::iterator end() { return path.end(); }
    EigenSTL::vector_Vector3d::const_iterator begin() const { return path.begin(); }
    EigenSTL::vector_Vector3d::const_iterator end() const { return path.end(); }
};

typedef std::vector<CartesianPath> MultiRobotPath;

class BioIKDemo
{
public:
    BioIKDemo();

    void moveHome();

    void planMultiRobotTrajectory(MultiRobotPath& paths);
    void setDefaultIKOptions(bio_ik::BioIKKinematicsQueryOptions& options) const;
    void createDensePaths(const MultiRobotPath& path_in, MultiRobotPath& path_out, double eef_step = 0.005) const;
    void execute();

private:
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    robot_trajectory::RobotTrajectoryPtr robot_trajectory_;
};

} // namespace hydra_demos

#endif // BIO_IK_DEMO_H
