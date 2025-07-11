#pragma once

#include <memory>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/grid_cells.hpp>

#include "ugv_nav4d_ros2/msg/mls_map.hpp"
#include "ugv_nav4d_ros2/msg/mls_patch.hpp"
#include "ugv_nav4d_ros2/action/save_mls_map.hpp"
#include "ugv_nav4d_ros2/msg/labeled_path_array.hpp"


#include "ugv_nav4d_ros2/msg/trav_map.hpp"
#include "ugv_nav4d_ros2/msg/trav_patch.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

#include <ugv_nav4d/Planner.hpp>
#include <maps/grid/MLSMap.hpp>


namespace ugv_nav4d_ros2{

class PathPlannerNode: public rclcpp::Node
{
public:
    PathPlannerNode();
    using SaveMLSMap = ugv_nav4d_ros2::action::SaveMLSMap;

private:
    void setupSubscriptions();
    bool read_pose_from_tf();
    void map_publish_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void process_goal_request(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void read_start_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    bool loadPlyAsMLS(const std::string& path);
    bool generateMLS();
    bool saveMLSMapAsBin(const std::string& filename);
    bool loadMLSMapFromBin(const std::string& filename);
    void plan();
    void declareParameters();
    void updateParameters();
    void configurePlanner();
    void publishTravMap();
    bool publishMLSMap();
    void parameterUpdateTimerCallback();

    //action server
    rclcpp_action::GoalResponse handle_save_map_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const SaveMLSMap::Goal> goal);
    rclcpp_action::CancelResponse handle_save_map_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SaveMLSMap>> goal_handle);
    void handle_save_map_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SaveMLSMap>> goal_handle);
    rclcpp_action::Server<SaveMLSMap>::SharedPtr save_mls_map_action_server;
   
    //parameters
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

    //subscriptions
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_start_pose;

    geometry_msgs::msg::PoseStamped start_pose;
    geometry_msgs::msg::PoseStamped goal_pose;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud;

    //publishers
    rclcpp::Publisher<ugv_nav4d_ros2::msg::LabeledPathArray>::SharedPtr labeled_path_publisher;
    rclcpp::Publisher<ugv_nav4d_ros2::msg::TravMap>::SharedPtr trav_map_publisher;
    rclcpp::Publisher<ugv_nav4d_ros2::msg::MLSMap>::SharedPtr mls_map_publisher;

    //services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_publish_service; 

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
    bool inPlanningPhase;
    bool gotMap;
    bool isConfigured;
    double mls_min_x;
    double mls_min_y;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    std::vector<rclcpp::Parameter> parameters_to_update;
    rclcpp::TimerBase::SharedPtr timer;
    pcl::CropBox<pcl::PointXYZ> box_filter;
    
    bool extend_trajectory_;
    double extension_distance_;

};

} // namespace ugv_nav4d_ros2 

