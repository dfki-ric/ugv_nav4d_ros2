#pragma once

#include <memory>
#include <Eigen/Dense>
#include <functional>
#include <atomic>
#include <mutex>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "ugv_nav4d_ros2/msg/mls_map.hpp"
#include "ugv_nav4d_ros2/msg/mls_patch.hpp"
#include "ugv_nav4d_ros2/action/save_mls_map.hpp"
#include "ugv_nav4d_ros2/msg/labeled_path_array.hpp"
#include "ugv_nav4d_ros2/msg/forbidden_zone.hpp"
#include "ugv_nav4d_ros2/msg/mission_status.hpp"
#include "ugv_nav4d_ros2/msg/route_risk.hpp"
#include "ugv_nav4d_ros2/msg/operational_zone_array.hpp"
#include "ugv_nav4d_ros2/srv/edit_waypoint.hpp"
#include "ugv_nav4d_ros2/srv/delete_forbidden_zone.hpp"
#include "ugv_nav4d_ros2/srv/inspect_traversability.hpp"
#include "ugv_nav4d_ros2/srv/mission_file.hpp"

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
    void regenerateMapsCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void processGoalRequest(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void readStartPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    //waypoints
    void addWaypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void clearWaypointsCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void reverseWaypointsCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void removeLastWaypointCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void publishWaypointMarkers();
    void editWaypointCallback(const std::shared_ptr<ugv_nav4d_ros2::srv::EditWaypoint::Request> request,
                              std::shared_ptr<ugv_nav4d_ros2::srv::EditWaypoint::Response> response);
    void planThroughWaypoints(bool record_mission = true);
    void publishStatus(const std::string& status);
    void publishMissionStatus(uint8_t state, const std::string& summary,
                              const std::string& failure_reason = "");
    void publishRouteRisk(const nav_msgs::msg::Path& path,
                          const ugv_nav4d_ros2::msg::LabeledPathArray& labeled);
    bool validatePendingPath();
    //Orientation inspection: render allowed-orientation wedges for all
    //partially traversable cells inside an operator-drawn region.
    void inspectOrientationsCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
    /** Clears the executing-route displays (combined path + mission markers). */
    void clearExecutingPathDisplay();
    void inspectTraversabilityCallback(
        const std::shared_ptr<ugv_nav4d_ros2::srv::InspectTraversability::Request> request,
        std::shared_ptr<ugv_nav4d_ros2::srv::InspectTraversability::Response> response);
    void replanCurrentMissionCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void recoverOutOfObstacleCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void saveMissionCallback(const std::shared_ptr<ugv_nav4d_ros2::srv::MissionFile::Request> request,
                             std::shared_ptr<ugv_nav4d_ros2::srv::MissionFile::Response> response);
    void loadMissionCallback(const std::shared_ptr<ugv_nav4d_ros2::srv::MissionFile::Request> request,
                             std::shared_ptr<ugv_nav4d_ros2::srv::MissionFile::Response> response);
    void saveMapCallback(const std::shared_ptr<ugv_nav4d_ros2::srv::MissionFile::Request> request,
                         std::shared_ptr<ugv_nav4d_ros2::srv::MissionFile::Response> response);
    void executePathCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void discardPathCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    //forbidden zones
    void addForbiddenZoneCallback(const ugv_nav4d_ros2::msg::ForbiddenZone::SharedPtr msg);
    void clearForbiddenZonesCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void removeLastForbiddenZoneCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void publishForbiddenZoneMarkers();
    void applyForbiddenZones();
    /** Flood-fill one zone onto the (already expanded) traversability map.
     *  KEEP_OUT nodes marked OBSTACLE are appended to @p keep_out_nodes so the
     *  caller can inflate a safety margin around them. Returns marked count. */
    size_t applyZoneToTravMap(const ugv_nav4d_ros2::msg::ForbiddenZone& zone,
                              std::vector<traversability_generator3d::TravGenNode*>* keep_out_nodes);
    /** Hard safety margin around zone obstacle cells, mirroring the radius logic
     *  of TraversabilityGenerator3d::inflateObstacles(). Returns inflated count. */
    size_t inflateZoneObstacles(const std::vector<traversability_generator3d::TravGenNode*>& seeds);
    /** @p added_zone enables the incremental fast path: a newly added zone is
     *  painted onto the existing map without the full configurePlanner rebuild. */
    void onForbiddenZonesChanged(bool planning_graph_changed,
                                 const ugv_nav4d_ros2::msg::ForbiddenZone* added_zone = nullptr);
    void deleteForbiddenZoneCallback(const std::shared_ptr<ugv_nav4d_ros2::srv::DeleteForbiddenZone::Request> request,
                                     std::shared_ptr<ugv_nav4d_ros2::srv::DeleteForbiddenZone::Response> response);
    void planReturnCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void planReturnCurrentCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void setReturnForwardCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                  std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    /** Measure the current wheel envelope from TF (the wheelbase is variable)
     *  and apply it as robotSizeX/robotSizeY plus configured body margins. */
    void updateFootprintCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    struct FootprintEnvelope
    {
        double wheel_min_x{0.0};   //!< wheel centers only: defines the wheelbase
        double wheel_max_x{0.0};
        double min_x{0.0};         //!< wheels + extra frames (tool/shovel)
        double max_x{0.0};
        double max_abs_y{0.0};
    };
    /** Measure wheel links and footprint_extra_frames (e.g. the tool mount)
     *  in the robot frame via TF. Returns false (with optional reason) when
     *  any transform is unavailable. */
    bool measureFootprintEnvelope(FootprintEnvelope& envelope, std::string* error = nullptr);
    /** Periodic: publish the measured wheelbase (Float32), the footprint
     *  polygon (like Nav2's published_footprint) and RViz markers. */
    void publishWheelbaseStatus();
    void publishPlannedPath(const std::vector<trajectory_follower::SubTrajectory>& trajectory3D, bool found_solution);
    static const char* planningResultToString(ugv_nav4d::Planner::PLANNING_RESULT res);
    void initializeMLSMap();
    bool loadPlyAsMLS(const std::string& path);
    bool generateMLS();
    bool saveMLSMapAsBin(const std::string& filename);
    bool loadMLSMapFromBin(const std::string& filename);
    void plan(bool record_mission = true);
    void declareParameters();
    void updateParameters();
    void configurePlanner();
    bool publishTravMap();
    bool publishMLSMap();
    bool publishMaps();
    void parameterUpdateTimerCallback();
    void poseUpdateTimerCallback();
    void rebuildSpeedZoneCache();
    void publishZoneSpeedLimit(const geometry_msgs::msg::Point& robot_position);
    void emitZoneSpeedLimit(float limit);

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
    //combined_path/mission_path show the EXECUTING route (published on Execute);
    //preview_path/preview_mission_path show the latest planned PREVIEW.
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr combined_path_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr preview_path_publisher;
    rclcpp::Publisher<ugv_nav4d_ros2::msg::LabeledPathArray>::SharedPtr labeled_path_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr colored_path_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr preview_colored_path_publisher;
    rclcpp::Publisher<ugv_nav4d_ros2::msg::TravMap>::SharedPtr trav_map_publisher;
    rclcpp::Publisher<ugv_nav4d_ros2::msg::MLSMap>::SharedPtr mls_map_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr zone_speed_limit_publisher;

    //services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_publish_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr regenerate_maps_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr update_footprint_service;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wheelbase_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr footprint_marker_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_polygon_publisher;
    rclcpp::TimerBase::SharedPtr wheelbase_status_timer;

    //waypoints
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_add_waypoint;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_waypoints_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr remove_last_waypoint_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reverse_waypoints_service;
    rclcpp::Service<ugv_nav4d_ros2::srv::EditWaypoint>::SharedPtr edit_waypoint_service;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_marker_publisher;
    std::vector<geometry_msgs::msg::Pose> waypoint_queue;

    //operator feedback
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher;
    rclcpp::Publisher<ugv_nav4d_ros2::msg::MissionStatus>::SharedPtr mission_status_publisher;
    rclcpp::Publisher<ugv_nav4d_ros2::msg::RouteRisk>::SharedPtr route_risk_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr route_valid_publisher;
    rclcpp::Service<ugv_nav4d_ros2::srv::MissionFile>::SharedPtr save_map_service;
    rclcpp::Service<ugv_nav4d_ros2::srv::InspectTraversability>::SharedPtr inspect_traversability_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replan_current_mission_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recover_out_of_obstacle_service;
    rclcpp::Service<ugv_nav4d_ros2::srv::MissionFile>::SharedPtr save_mission_service;
    rclcpp::Service<ugv_nav4d_ros2::srv::MissionFile>::SharedPtr load_mission_service;

    //execution gate: planned paths are only previews until confirmed
    rclcpp::Publisher<ugv_nav4d_ros2::msg::LabeledPathArray>::SharedPtr execute_path_publisher;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_path_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr discard_path_service;
    ugv_nav4d_ros2::msg::LabeledPathArray pending_labeled_path;
    //Executing-style rendering of the pending path, held back until Execute
    //promotes it onto the executing display topics.
    nav_msgs::msg::Path pending_display_path;
    visualization_msgs::msg::MarkerArray pending_display_markers;
    bool has_pending_path{false};
    bool path_approved{false};
    bool pending_path_is_recovery{false};
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr inspect_orientations_sub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr allowed_orientation_marker_publisher;

    //forbidden zones (keep-out): applied to the trav map after every regeneration
    rclcpp::Subscription<ugv_nav4d_ros2::msg::ForbiddenZone>::SharedPtr sub_add_forbidden_zone;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_forbidden_zones_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr remove_last_forbidden_zone_service;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr forbidden_zone_marker_publisher;
    rclcpp::Publisher<ugv_nav4d_ros2::msg::OperationalZoneArray>::SharedPtr operational_zones_publisher;
    rclcpp::Service<ugv_nav4d_ros2::srv::DeleteForbiddenZone>::SharedPtr delete_forbidden_zone_service;
    std::vector<ugv_nav4d_ros2::msg::ForbiddenZone> forbidden_zones;
    std::vector<std::unordered_set<const traversability_generator3d::TravGenNode*>> speed_zone_node_sets;
    float last_zone_speed_limit{0.0f};
    rclcpp::Time last_zone_speed_limit_match;

    // Return mission: endpoints of the last outward mission. Keeping both
    // makes return planning independent of the robot's live pose.
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_return_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_return_current_service;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_return_forward_service;
    base::samples::RigidBodyState last_mission_start;
    base::samples::RigidBodyState last_mission_goal;
    bool has_last_mission_start{false};
    //Mission anchor candidates: stashed at plan time, committed to last_mission_*
    //only when the plan is actually Executed. A discarded preview must not move
    //the return-home anchor.
    base::samples::RigidBodyState pending_mission_start;
    base::samples::RigidBodyState pending_mission_goal;
    bool pending_records_mission{false};
    bool is_return_plan{false};

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
