#include <hydra_demos/hydra_demo_base.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <chrono>

#include "mrmf_core/composite_interpolator.h"


using namespace mrmf_core;

namespace hydra_demos
{
class MrmfDemo : public HydraDemo
{
public:

    MrmfDemo() = default;

    void run()
    {
        globalSquares();
        //localSquares();
    }

    void globalSquares()
    {
        // create waypoints of square
        Eigen::Vector3d r1_a = {-0.300, -0.300, 0.0 };
        Eigen::Vector3d r1_b = {-0.300,  0.300, 0.0 };
        Eigen::Vector3d r1_c = { 0.300,  0.300, 0.0 };
        Eigen::Vector3d r1_d = { 0.300, -0.300, 0.0 };

        Eigen::Vector3d r2_a = { 0.275,  0.275, 0.0 };
        Eigen::Vector3d r2_b = { 0.275, -0.275, 0.0 };
        Eigen::Vector3d r2_c = {-0.275, -0.275, 0.0 };
        Eigen::Vector3d r2_d = {-0.275,  0.275, 0.0 };

        Eigen::Vector3d r3_a = {-0.250,  0.250, 0.0 };
        Eigen::Vector3d r3_b = { 0.250,  0.250, 0.0 };
        Eigen::Vector3d r3_c = { 0.250, -0.250, 0.0 };
        Eigen::Vector3d r3_d = {-0.250, -0.250, 0.0 };

        // create individual robot trajectories
        CartesianTrajectoryPtr rob1_traj(new CartesianTrajectory(robot1, 0.10, positioner));
        CartesianTrajectoryPtr rob2_traj(new CartesianTrajectory(robot2, 0.10, positioner));
        CartesianTrajectoryPtr rob3_traj(new CartesianTrajectory(robot3, 0.10, positioner));

        rob1_traj->addSuffixWayPoint(makeCartesianWaypoint(r1_a));
        rob1_traj->addSuffixWayPoint(makeCartesianWaypoint(r1_b));
        rob1_traj->addSuffixWayPoint(makeCartesianWaypoint(r1_c));
        rob1_traj->addSuffixWayPoint(makeCartesianWaypoint(r1_d));
        rob1_traj->addSuffixWayPoint(makeCartesianWaypoint(r1_a));

        rob2_traj->addSuffixWayPoint(makeCartesianWaypoint(r2_a));
        rob2_traj->addSuffixWayPoint(makeCartesianWaypoint(r2_b));
        rob2_traj->addSuffixWayPoint(makeCartesianWaypoint(r2_c));
        rob2_traj->addSuffixWayPoint(makeCartesianWaypoint(r2_d));
        rob2_traj->addSuffixWayPoint(makeCartesianWaypoint(r2_a));

        rob3_traj->addSuffixWayPoint(makeCartesianWaypoint(r3_a));
        rob3_traj->addSuffixWayPoint(makeCartesianWaypoint(r3_b));
        rob3_traj->addSuffixWayPoint(makeCartesianWaypoint(r3_c));
        rob3_traj->addSuffixWayPoint(makeCartesianWaypoint(r3_d));
        rob3_traj->addSuffixWayPoint(makeCartesianWaypoint(r3_a));

        // create synchronized trajectory
        SynchronousTrajectory sync_traj({rob1_traj, rob2_traj, rob3_traj});
        planAndExecute(sync_traj);    
    }

    void localSquares()
    {
        // create waypoints of squares
        Eigen::Vector3d r1_a = {-0.19913, -0.290, 0.0};
        Eigen::Vector3d r1_b = {-0.19913, -0.140, 0.0};
        Eigen::Vector3d r1_c = {-0.04913, -0.140, 0.0};
        Eigen::Vector3d r1_d = {-0.04913, -0.290, 0.0};

        Eigen::Vector3d r2_a = {0.32326,  0.075, 0.0};
        Eigen::Vector3d r2_b = {0.32326, -0.075, 0.0};
        Eigen::Vector3d r2_c = {0.17326, -0.075, 0.0};
        Eigen::Vector3d r2_d = {0.17326,  0.075, 0.0};

        Eigen::Vector3d r3_a = {-0.19913, 0.290, 0.0};
        Eigen::Vector3d r3_b = {-0.04913, 0.290, 0.0};
        Eigen::Vector3d r3_c = {-0.04913, 0.140, 0.0};
        Eigen::Vector3d r3_d = {-0.19913, 0.140, 0.0};      

        // create individual robot trajectories
        CartesianTrajectoryPtr rob1_traj(new CartesianTrajectory(robot1, 0.300, positioner));
        CartesianTrajectoryPtr rob2_traj(new CartesianTrajectory(robot2, 0.300, positioner));
        CartesianTrajectoryPtr rob3_traj(new CartesianTrajectory(robot3, 0.300, positioner));

        rob1_traj->addSuffixWayPoint(makeCartesianWaypoint(r1_a));
        rob1_traj->addSuffixWayPoint(makeCartesianWaypoint(r1_b));
        rob1_traj->addSuffixWayPoint(makeCartesianWaypoint(r1_c));
        rob1_traj->addSuffixWayPoint(makeCartesianWaypoint(r1_d));
        rob1_traj->addSuffixWayPoint(makeCartesianWaypoint(r1_a));

        rob2_traj->addSuffixWayPoint(makeCartesianWaypoint(r2_a));
        rob2_traj->addSuffixWayPoint(makeCartesianWaypoint(r2_b));
        rob2_traj->addSuffixWayPoint(makeCartesianWaypoint(r2_c));
        rob2_traj->addSuffixWayPoint(makeCartesianWaypoint(r2_d));
        rob2_traj->addSuffixWayPoint(makeCartesianWaypoint(r2_a));

        rob3_traj->addSuffixWayPoint(makeCartesianWaypoint(r3_a));
        rob3_traj->addSuffixWayPoint(makeCartesianWaypoint(r3_b));
        rob3_traj->addSuffixWayPoint(makeCartesianWaypoint(r3_c));
        rob3_traj->addSuffixWayPoint(makeCartesianWaypoint(r3_d));
        rob3_traj->addSuffixWayPoint(makeCartesianWaypoint(r3_a));

        // create synchronized trajectory
        SynchronousTrajectory sync_traj({rob1_traj, rob2_traj, rob3_traj});
        planAndExecute(sync_traj);    
    }

    void planAndExecute(SynchronousTrajectory& sync_traj)
    {
        // plan for multi-robot trajectory
        robot_trajectory::RobotTrajectory output_traj(move_group_interface_.getRobotModel(), PLANNING_GROUP);

        auto start = std::chrono::high_resolution_clock::now();
        bool success = multi_robot_group_.planMultiRobotTrajectory2(sync_traj, output_traj, 
            *(move_group_interface_.getCurrentState()));
        auto stop = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        std::cout << "Took " << duration.count() << " us to plan traj" << std::endl;

        if (success)
        {
            // convert to trajectory message
            moveit_msgs::RobotTrajectory trajectory_msg;
            output_traj.getRobotTrajectoryMsg(trajectory_msg);

            // execute
            moveHome();
            move_group_interface_.execute(trajectory_msg);
            moveHome();
        }
        else
        {
            ROS_ERROR("Failed to plan synchronous trajectory");
            exit(-1);
        } 
    }

private:
CartesianWaypointPtr makeCartesianWaypoint(const Eigen::Vector3d& point)
{
    // make all poses point toward the center
    double rotation_angle = -atan2(point.y(), point.x()) - M_PI;

    Eigen::Isometry3d pose = Eigen::Translation3d(point) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(rotation_angle, Eigen::Vector3d::UnitZ());

    return CartesianWaypointPtr(new CartesianWaypoint(pose));
}

};

} // namespace hydra_demos 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mrmf_demo_node");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    hydra_demos::MrmfDemo application;
    application.run();

    return 0;
}