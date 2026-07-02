#pragma once

#include <memory>
#include <Eigen/Dense>
#include <functional>
#include <atomic>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>

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
#include <traversability_generator3d/TraversabilityGenerator3d.hpp>
#include <maps/grid/MLSMap.hpp>


namespace ugv_nav4d_ros2{

class PathPlannerNode: public rclcpp::Node
{
public:
    PathPlannerNode();
    using SaveMLSMap = ugv_nav4d_ros2::action::SaveMLSMap;

    // Getters and callback registration for standalone GUI integration
    std::shared_ptr<maps::grid::MLSMapSloped> getMLSMap() const { return mls_map_ptr; }
    std::shared_ptr<const traversability_generator3d::TravMap3d> getTraversabilityMap() const {
        return planner_ptr ? planner_ptr->getTraversabilityMap() : nullptr;
    }
    ugv_nav4d::Planner* getPlanner() const { return planner_ptr.get(); }

    // Load configuration from parameters (populate structs from YAML)
    void loadConfigsFromParameters() { updateParameters(); }

    // Push configs (e.g. edited in the GUI) into the ROS 2 parameter server.
    // This triggers parametersCallback -> timer -> configurePlanner(), rebuilding the planner.
    void updateParametersFromConfigs(
        const sbpl_spline_primitives::SplinePrimitivesConfig& spline,
        const ugv_nav4d::Mobility& mobility,
        const traversability_generator3d::TraversabilityConfig& trav,
        const ugv_nav4d::PlannerConfig& planner);

    // Get planner configurations (for GUI to read parameters from node)
    const sbpl_spline_primitives::SplinePrimitivesConfig& getSplineConfig() const { return spline_primitive_config; }
    const ugv_nav4d::Mobility& getMobilityConfig() const { return mobility_config; }
    const traversability_generator3d::TraversabilityConfig& getTraversabilityConfig() const { return traversability_config; }
    const ugv_nav4d::PlannerConfig& getPlannerConfig() const { return planner_config; }
    // Register a callback that receives short status messages (e.g. "Loading map...", "Ready")
    // for display in a GUI.
    void registerStatusCallback(std::function<void(const std::string&)> cb) {
        status_callback = cb;
    }
    // Register a callback that receives the robot start pose at a fixed rate (from TF or the
    // /start_pose topic), so a GUI can continuously track the robot pose.
    void registerPoseUpdateCallback(std::function<void(const base::Pose&)> cb) {
        pose_update_callback = cb;
    }
    void registerMapUpdateCallback(std::function<void()> cb) {
        map_update_callback = cb;
        if (got_map && map_update_callback) {
            map_update_callback();
        }
    }
    void triggerPlanningFromGUI(const base::Pose& start, const base::Pose& goal);
    // Robot start pose as known to the node (from TF or the /start_pose topic), for GUI display.
    base::Pose getStartPose() const {
        return base::Pose(
            Eigen::Vector3d(start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z),
            Eigen::Quaterniond(start_pose.pose.orientation.w, start_pose.pose.orientation.x,
                               start_pose.pose.orientation.y, start_pose.pose.orientation.z));
    }
    const std::vector<trajectory_follower::SubTrajectory>& getLatestTrajectory2D() const { return latest_trajectory2D; }
    const std::vector<trajectory_follower::SubTrajectory>& getLatestTrajectory3D() const { return latest_trajectory3D; }
    ugv_nav4d::Planner::PLANNING_RESULT getLatestPlanningResult() const { return latest_planning_result; }

private:
    void setupSubscriptions();
    bool updatePoseFromTF();
    void mapPublishCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void processGoalRequest(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void readStartPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void initializeMLSMap();
    bool loadPlyAsMLS(const std::string& path);
    bool generateMLS();
    bool saveMLSMapAsBin(const std::string& filename);
    bool loadMLSMapFromBin(const std::string& filename);
    void plan();
    void declareParameters();
    void updateParameters();
    void configurePlanner();
    bool publishTravMap();
    bool publishMLSMap();
    bool publishMaps();
    void parameterUpdateTimerCallback();
    void poseUpdateTimerCallback();

    //action server
    rclcpp_action::GoalResponse actionSaveMap(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const SaveMLSMap::Goal> goal);
    rclcpp_action::CancelResponse actionCancelSaveMap(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SaveMLSMap>> goal_handle);
    void actionSaveMapAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SaveMLSMap>> goal_handle);
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
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr combined_path_publisher;
    rclcpp::Publisher<ugv_nav4d_ros2::msg::LabeledPathArray>::SharedPtr labeled_path_publisher;
    rclcpp::Publisher<ugv_nav4d_ros2::msg::TravMap>::SharedPtr trav_map_publisher;
    rclcpp::Publisher<ugv_nav4d_ros2::msg::MLSMap>::SharedPtr mls_map_publisher;

    //services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_publish_service; 

    //tf
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ptr;

    //planner
    sbpl_spline_primitives::SplinePrimitivesConfig spline_primitive_config; 
    ugv_nav4d::Mobility mobility_config;
    traversability_generator3d::TraversabilityConfig traversability_config;
    ugv_nav4d::PlannerConfig planner_config;

    std::unique_ptr<ugv_nav4d::Planner> planner_ptr;
    std::shared_ptr<traversability_generator3d::TraversabilityGenerator3d> traversability_generator_ptr;
    std::shared_ptr<traversability_generator3d::TraversabilityGenerator3d::MLGrid> mls_map_ptr;
    base::samples::RigidBodyState start_pose_rbs;
    base::samples::RigidBodyState goal_pose_rbs;

    bool initial_patch_added;
    std::atomic<bool> is_planning;
    bool got_map;
    bool is_configured;
    double mls_min_x;
    double mls_min_y;
    double current_grid_resolution; // resolution the current MLS map was built with

    // Serializes all access to the planner / traversability generator / MLS map, which are
    // touched by the ROS executor thread (timer, subscriptions) and the GUI planning thread.
    // Recursive so entry points may lock and still call configurePlanner()/plan() safely.
    std::recursive_mutex planner_mutex;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    std::vector<rclcpp::Parameter> parameters_to_update;
    rclcpp::TimerBase::SharedPtr timer;
    pcl::CropBox<pcl::PointXYZ> box_filter;
    
    bool extend_trajectory;
    double extension_distance;

    std::function<void()> map_update_callback;
    std::function<void(const std::string&)> status_callback;
    std::function<void(const base::Pose&)> pose_update_callback;
    rclcpp::TimerBase::SharedPtr pose_update_timer;
    std::vector<trajectory_follower::SubTrajectory> latest_trajectory2D;
    std::vector<trajectory_follower::SubTrajectory> latest_trajectory3D;
    ugv_nav4d::Planner::PLANNING_RESULT latest_planning_result;
};

} // namespace ugv_nav4d_ros2 

