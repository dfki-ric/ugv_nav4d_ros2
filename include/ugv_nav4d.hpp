#pragma once

#include <memory>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <ugv_nav4d/Planner.hpp>
#include <maps/grid/MLSMap.hpp>

namespace ugv_nav4d_ros2{

class PathPlannerNode: public rclcpp::Node
{
public:
    PathPlannerNode();

private:
    bool read_pose_samples();
    void process_goal_request(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void printPlannerConfig();
    bool loadMls(const std::string& path);
    void plan();
    void declareParameters();
    void updateParameters();
    void configurePlanner();

    //subscriptions
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose;
    rclcpp::TimerBase::SharedPtr timer_pose_samples;
    geometry_msgs::msg::PoseStamped pose_samples;
    geometry_msgs::msg::PoseStamped goal_pose;

    //publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;

    //tf
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    //planner
    sbpl_spline_primitives::SplinePrimitivesConfig splinePrimitiveConfig; 
    ugv_nav4d::Mobility mobility;
    traversability_generator3d::TraversabilityConfig traversabilityConfig;
    ugv_nav4d::PlannerConfig plannerConfig;

    std::unique_ptr<ugv_nav4d::Planner> planner;
    base::samples::RigidBodyState start_pose_rbs;
    base::samples::RigidBodyState goal_pose_rbs;

    maps::grid::MLSMapSloped mlsMap;
    bool initialPatchAdded;
};

} // namespace ugv_nav4d_ros2 

