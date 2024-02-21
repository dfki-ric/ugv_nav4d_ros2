#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


#include <ugv_nav4d/Planner.hpp>

class PathPlannerNode: public rclcpp::Node
{
public:
    PathPlannerNode();

private:
    bool read_pose_samples();
    void process_goal_request(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose;
    rclcpp::TimerBase::SharedPtr timer_pose_samples;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    sbpl_spline_primitives::SplinePrimitivesConfig splinePrimitiveConfig; 
    ugv_nav4d::Mobility mobility;
    traversability_generator3d::TraversabilityConfig traversabilityConfig;
    ugv_nav4d::PlannerConfig plannerConfig;

    geometry_msgs::msg::PoseStamped pose_samples;
    geometry_msgs::msg::PoseStamped goal_pose;

    std::string robot_frame;
    std::string map_frame;

    base::Time maxTime;
    double initialPatchRadius;
    double recoveryTimeOut;
    int dumpOnError;
    int dumpOnSuccess;
};

