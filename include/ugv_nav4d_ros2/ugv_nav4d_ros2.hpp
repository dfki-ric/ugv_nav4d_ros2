#pragma once

#include <memory>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/grid_cells.hpp>

#include "ugv_nav4d_ros2/msg/mls_map.hpp"
#include "ugv_nav4d_ros2/msg/mls_patch.hpp"

#include "ugv_nav4d_ros2/msg/trav_map.hpp"
#include "ugv_nav4d_ros2/msg/trav_patch.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ugv_nav4d/Planner.hpp>
#include <maps/grid/MLSMap.hpp>


namespace ugv_nav4d_ros2{

class PathPlannerNode: public rclcpp::Node
{
public:
    PathPlannerNode();

private:
    bool pose_samples_callback();
    void map_publish_callback();
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void process_goal_request(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void printPlannerConfig();
    bool loadMls(const std::string& path);
    bool generateMls();
    void plan();
    void declareParameters();
    void updateParameters();
    void configurePlanner();
    void publishTravMap();
    bool publishMLSMap();

    //subscriptions
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose;
    rclcpp::TimerBase::SharedPtr timer_pose_samples;
    rclcpp::TimerBase::SharedPtr timer_map_publish;

    geometry_msgs::msg::PoseStamped pose_samples;
    geometry_msgs::msg::PoseStamped goal_pose;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud;

    //publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
    rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr grid_map_publisher;
    rclcpp::Publisher<ugv_nav4d_ros2::msg::TravMap>::SharedPtr trav_map_publisher;
    rclcpp::Publisher<ugv_nav4d_ros2::msg::MLSMap>::SharedPtr mls_map_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_map_publisher;

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

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    
};

} // namespace ugv_nav4d_ros2 

