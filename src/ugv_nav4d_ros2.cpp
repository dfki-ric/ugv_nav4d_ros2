#include "ugv_nav4d_ros2.hpp"
#include "util_functions.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h> // For removeNaNFromPointCloud
#include <pcl/filters/voxel_grid.h> // For optional PLY downsampling

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <algorithm>
#include <fstream>
#include <cmath>
#include <chrono>
#include <deque>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unordered_set>

using namespace rclcpp;

namespace ugv_nav4d_ros2 {

PathPlannerNode::PathPlannerNode()
    : Node("ugv_nav4d_ros2")
    , initial_patch_added(false)
    , is_planning(false)
    , got_map(false)
    , is_configured(false)
{
    declareParameters();

    Eigen::Vector4f min_point;
    Eigen::Vector4f max_point;

    min_point.x() = get_parameter("dist_min_x").as_int();
    min_point.y() = get_parameter("dist_min_y").as_int(); 
    min_point.z() = get_parameter("dist_min_z").as_double();

    max_point.x() = get_parameter("dist_max_x").as_int();
    max_point.y() = get_parameter("dist_max_y").as_int(); 
    max_point.z() = get_parameter("dist_max_z").as_double();

    box_filter.setMin(min_point);  // Set minimum bound
    box_filter.setMax(max_point);  // Set maximum bound
    
    mls_min_x = get_parameter("dist_min_x").as_int();
    mls_min_y = get_parameter("dist_min_y").as_int();

    extend_trajectory = get_parameter("extend_trajectory").as_bool();
    extension_distance = get_parameter("extension_distance").as_double();

    initializeMLSMap();

    combined_path_publisher = this->create_publisher<nav_msgs::msg::Path>("/ugv_nav4d_ros2/path", 10);
    labeled_path_publisher = this->create_publisher<ugv_nav4d_ros2::msg::LabeledPathArray>("/ugv_nav4d_ros2/labeled_path_segments", 10);
    colored_path_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/ugv_nav4d_ros2/mission_path", rclcpp::QoS(1).transient_local());
    trav_map_publisher = this->create_publisher<ugv_nav4d_ros2::msg::TravMap>("/ugv_nav4d_ros2/trav_map", 10);
    mls_map_publisher = this->create_publisher<ugv_nav4d_ros2::msg::MLSMap>("/ugv_nav4d_ros2/mls_map", 10);

    setupSubscriptions();
}

void PathPlannerNode::setupSubscriptions()
{
    // controller feedback (via TF)
    tf_buffer_ptr = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ptr = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr);

    // Create the action server
    save_mls_map_action_server = rclcpp_action::create_server<SaveMLSMap>(
        this,
        "/ugv_nav4d_ros2/save_mls_map",
        std::bind(&PathPlannerNode::actionSaveMap, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PathPlannerNode::actionCancelSaveMap, this, std::placeholders::_1),
        std::bind(&PathPlannerNode::actionSaveMapAccepted, this, std::placeholders::_1)
    );

    // Map publisher trigger service
    map_publish_service = this->create_service<std_srvs::srv::Trigger>(
            "/ugv_nav4d_ros2/map_publish", std::bind(&PathPlannerNode::mapPublishCallback, this, std::placeholders::_1, std::placeholders::_2));
    regenerate_maps_service = this->create_service<std_srvs::srv::Trigger>(
            "/ugv_nav4d_ros2/regenerate_maps", std::bind(&PathPlannerNode::regenerateMapsCallback, this, std::placeholders::_1, std::placeholders::_2));

    sub_goal_pose = create_subscription<geometry_msgs::msg::PoseStamped>("/ugv_nav4d_ros2/goal_pose", 1,
            bind(&PathPlannerNode::processGoalRequest, this, std::placeholders::_1));

    // Waypoint queue: poses accumulated here are planned through (in order) when the
    // next goal arrives. The queue is only consumed by planning, never by time.
    sub_add_waypoint = create_subscription<geometry_msgs::msg::PoseStamped>("/ugv_nav4d_ros2/add_waypoint", 10,
            bind(&PathPlannerNode::addWaypointCallback, this, std::placeholders::_1));

    clear_waypoints_service = this->create_service<std_srvs::srv::Trigger>(
            "/ugv_nav4d_ros2/clear_waypoints", std::bind(&PathPlannerNode::clearWaypointsCallback, this, std::placeholders::_1, std::placeholders::_2));

    remove_last_waypoint_service = this->create_service<std_srvs::srv::Trigger>(
            "/ugv_nav4d_ros2/remove_last_waypoint", std::bind(&PathPlannerNode::removeLastWaypointCallback, this, std::placeholders::_1, std::placeholders::_2));

    edit_waypoint_service = this->create_service<ugv_nav4d_ros2::srv::EditWaypoint>(
            "/ugv_nav4d_ros2/edit_waypoint", std::bind(&PathPlannerNode::editWaypointCallback, this, std::placeholders::_1, std::placeholders::_2));

    // transient_local so RViz still shows the queued waypoints after a display restart
    waypoint_marker_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/ugv_nav4d_ros2/waypoint_markers", rclcpp::QoS(1).transient_local());

    // Operator feedback: latest planner status as a plain string (transient_local so a
    // late-joining RViz panel immediately sees the current state).
    status_publisher = this->create_publisher<std_msgs::msg::String>(
            "/ugv_nav4d_ros2/status", rclcpp::QoS(1).transient_local());

    save_map_service = this->create_service<std_srvs::srv::Trigger>(
            "/ugv_nav4d_ros2/save_mls_map", std::bind(&PathPlannerNode::saveMapCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Execution gate: planning only publishes a preview (path / labeled_path_segments /
    // colored_path). The follower listens on execute_path_segments, which is only
    // published when the operator confirms via the execute_path service.
    execute_path_publisher = this->create_publisher<ugv_nav4d_ros2::msg::LabeledPathArray>(
            "/ugv_nav4d_ros2/execute_path_segments", 10);

    execute_path_service = this->create_service<std_srvs::srv::Trigger>(
            "/ugv_nav4d_ros2/execute_path", std::bind(&PathPlannerNode::executePathCallback, this, std::placeholders::_1, std::placeholders::_2));

    discard_path_service = this->create_service<std_srvs::srv::Trigger>(
            "/ugv_nav4d_ros2/discard_path", std::bind(&PathPlannerNode::discardPathCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Forbidden zones (keep-out): circular regions the planner must not enter.
    sub_add_forbidden_zone = create_subscription<ugv_nav4d_ros2::msg::ForbiddenZone>(
            "/ugv_nav4d_ros2/add_forbidden_zone", 10,
            bind(&PathPlannerNode::addForbiddenZoneCallback, this, std::placeholders::_1));

    clear_forbidden_zones_service = this->create_service<std_srvs::srv::Trigger>(
            "/ugv_nav4d_ros2/clear_forbidden_zones", std::bind(&PathPlannerNode::clearForbiddenZonesCallback, this, std::placeholders::_1, std::placeholders::_2));

    remove_last_forbidden_zone_service = this->create_service<std_srvs::srv::Trigger>(
            "/ugv_nav4d_ros2/remove_last_forbidden_zone", std::bind(&PathPlannerNode::removeLastForbiddenZoneCallback, this, std::placeholders::_1, std::placeholders::_2));

    forbidden_zone_marker_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/ugv_nav4d_ros2/forbidden_zone_markers", rclcpp::QoS(1).transient_local());

    delete_forbidden_zone_service = this->create_service<ugv_nav4d_ros2::srv::DeleteForbiddenZone>(
            "/ugv_nav4d_ros2/delete_forbidden_zone", std::bind(&PathPlannerNode::deleteForbiddenZoneCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Plan back to where the last mission started, visiting the waypoints in
    // reverse order.
    plan_return_service = this->create_service<std_srvs::srv::Trigger>(
            "/ugv_nav4d_ros2/plan_return", std::bind(&PathPlannerNode::planReturnCallback, this, std::placeholders::_1, std::placeholders::_2));
    set_return_forward_service = this->create_service<std_srvs::srv::SetBool>(
            "/ugv_nav4d_ros2/set_return_forward", std::bind(&PathPlannerNode::setReturnForwardCallback, this, std::placeholders::_1, std::placeholders::_2));

    sub_start_pose = create_subscription<geometry_msgs::msg::PoseStamped>("/ugv_nav4d_ros2/start_pose", 1, 
            bind(&PathPlannerNode::readStartPose, this, std::placeholders::_1));

    parameter_callback_handle = this->add_on_set_parameters_callback(
        std::bind(&PathPlannerNode::parametersCallback, this, std::placeholders::_1));

    timer = this->create_wall_timer(
        std::chrono::milliseconds(1000),  // Timer period
        std::bind(&PathPlannerNode::parameterUpdateTimerCallback, this)  // Callback function
    );

    // Periodically push the robot start pose (TF / topic) so a GUI can track it continuously.
    pose_update_timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PathPlannerNode::poseUpdateTimerCallback, this)
    );

    if (!get_parameter("load_mls_from_file").as_bool()){
        cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/ugv_nav4d_ros2/pointcloud", 10,
                std::bind(&PathPlannerNode::cloudCallback, this, std::placeholders::_1));
    }
}

void PathPlannerNode::parameterUpdateTimerCallback(){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    // parameters_to_update is only used as a "dirty" flag: the values were already applied
    // by the framework when set. Re-setting them here would re-enter parametersCallback and
    // mutate this vector mid-iteration, so we only reconfigure.
    if (!parameters_to_update.empty())
    {
        parameters_to_update.clear();
        RCLCPP_INFO(this->get_logger(), "Parameters changed; reconfiguring planner.");
        configurePlanner();
    }
}

void PathPlannerNode::poseUpdateTimerCallback(){
    if (!pose_update_callback){
        return;
    }

    base::Pose pose;
    {
        std::lock_guard<std::recursive_mutex> lock(planner_mutex);
        // In topic mode the pose is kept current by readStartPose(); otherwise refresh from TF.
        if (!get_parameter("read_pose_from_topic").as_bool()){
            if (!updatePoseFromTF()){
                return; // no transform available yet; skip this tick (no log spam)
            }
        }
        pose = getStartPose();
    }
    pose_update_callback(pose);
}

rcl_interfaces::msg::SetParametersResult PathPlannerNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters){
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param: parameters)
    {
        if (this->has_parameter(param.get_name()))
        {
            // This is an operational return-path preference, not planner or
            // traversability configuration. It is read directly when Plan
            // Return is requested, so applying it must not rebuild the planner
            // or regenerate the traversability map.
            if (param.get_name() == "return_face_forward")
            {
                continue;
            }
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                parameters_to_update.push_back(rclcpp::Parameter(param.get_name(), param.as_int()));
            }
            else if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                parameters_to_update.push_back(rclcpp::Parameter(param.get_name(), param.as_string()));
            }
            else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                parameters_to_update.push_back(rclcpp::Parameter(param.get_name(), param.as_double()));
            }
            else if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
            {
                parameters_to_update.push_back(rclcpp::Parameter(param.get_name(), param.as_bool()));
            }
        }
        else
        {
            // Skip the unknown parameter but do not fail the whole batch, otherwise one bad
            // name would cause every other (valid) change in the same set to be rejected.
            RCLCPP_WARN(this->get_logger(), "Ignoring unknown parameter '%s'.", param.get_name().c_str());
        }
    }

    return result;
}

void PathPlannerNode::mapPublishCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!got_map)
    {
        RCLCPP_WARN(this->get_logger(), "Cannot publish maps: no map loaded yet (no pointcloud received or file not loaded).");
        response->success = false;
        response->message = "No map received so far.";
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Received service request to publish maps.");

    if (publishMaps())
    {
        response->success = true;
        response->message = "Published MLS and Traversability Map.";
        RCLCPP_INFO(this->get_logger(), "Published MLS and Traversability Map.");
    }
    else
    {
        response->success = false;
        response->message = "Failed to publish maps.";
        RCLCPP_ERROR(this->get_logger(), "Failed to publish maps.");
    }
}

void PathPlannerNode::regenerateMapsCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (is_planning)
    {
        response->success = false;
        response->message = "Cannot regenerate maps while planning is active.";
        publishStatus(response->message);
        return;
    }

    if (!get_parameter("read_pose_from_topic").as_bool() && !updatePoseFromTF())
    {
        response->success = false;
        response->message = "Cannot regenerate maps: TF lookup of the robot pose failed.";
        publishStatus(response->message);
        return;
    }

    publishStatus("Regenerating MLS and traversability maps...");
    if (get_parameter("load_mls_from_file").as_bool())
    {
        got_map = false;
        initializeMLSMap();
    }
    else
    {
        if (!latest_pointcloud)
        {
            response->success = false;
            response->message = "Cannot regenerate maps: no point cloud has been received.";
            publishStatus(response->message);
            return;
        }
        initializeMLSMap();
        got_map = generateMLS();
    }

    if (!got_map || !mls_map_ptr)
    {
        response->success = false;
        response->message = "MLS regeneration failed; check the configured map source.";
        publishStatus(response->message);
        return;
    }

    is_configured = false;
    configurePlanner();
    response->success = publishMLSMap() && publishTravMap();
    response->message = response->success
        ? "Regenerated and published MLS and traversability maps from the current robot pose."
        : "Maps regenerated, but publishing failed.";
    publishStatus(response->message);
}

void PathPlannerNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    latest_pointcloud = msg;
    got_map = generateMLS();

    if (got_map){

        if (!is_configured){
            configurePlanner();
        }

        if (!is_planning){
            if (!get_parameter("read_pose_from_topic").as_bool())
            {
                if (!updatePoseFromTF()){
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot determine start pose: TF lookup "
                                << get_parameter("world_frame").as_string() << " <- "
                                << get_parameter("robot_frame").as_string() << " failed.");
                    return;
                }
            }

            RCLCPP_INFO_STREAM(this->get_logger(), "Planner state: Got map (pointcloud with "
                               << latest_pointcloud->width * latest_pointcloud->height << " points merged into MLS).");
            traversability_generator_ptr->setMLSGrid(mls_map_ptr);

            Eigen::Affine3d body2MLS;
            body2MLS.translation() << start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z;
            Eigen::Quaterniond quat(start_pose.pose.orientation.w, 
                                    start_pose.pose.orientation.x, 
                                    start_pose.pose.orientation.y, 
                                    start_pose.pose.orientation.z);
            body2MLS.linear() = quat.toRotationMatrix(); 

            Eigen::Affine3d ground2Body(Eigen::Affine3d::Identity());
            ground2Body.translation() = Eigen::Vector3d(0, 0, -get_parameter("distToGround").as_double());

            Eigen::Affine3d ground2Mls(body2MLS * ground2Body);

            const double& initial_patch_radius = get_parameter("initialPatchRadius").as_double();

            if (!initial_patch_added && initial_patch_radius > 0.0){
                traversability_generator_ptr->setInitialPatch(ground2Mls, get_parameter("initialPatchRadius").as_double());
                initial_patch_added = true;
                RCLCPP_INFO_STREAM(this->get_logger(), "Initial patch (radius " << initial_patch_radius << " m) added to MLS at the robot position.");
            }

            auto startPosition = ground2Mls.translation();
            traversability_generator_ptr->expandAll(startPosition);
            applyForbiddenZones();
            auto travMap = traversability_generator_ptr->getTraversabilityMap();
            planner_ptr->updateMap(travMap);
            RCLCPP_INFO(this->get_logger(), "Planner state: Ready");
            publishStatus("Ready");
            if (map_update_callback) {
                map_update_callback();
            }
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Skipping map update: planning is in progress.");
        }
    }
    else{
        RCLCPP_WARN_STREAM(this->get_logger(), "Failed to build MLS from the incoming pointcloud.");
    }
}

bool PathPlannerNode::updatePoseFromTF(){
    std::string robot_frame = get_parameter("robot_frame").as_string();
    std::string world_frame = get_parameter("world_frame").as_string();

    try{
        geometry_msgs::msg::TransformStamped t = tf_buffer_ptr->lookupTransform(world_frame, robot_frame, tf2::TimePointZero);
        start_pose.pose.orientation = t.transform.rotation;
        start_pose.pose.position.x =  t.transform.translation.x;
        start_pose.pose.position.y =  t.transform.translation.y;
        start_pose.pose.position.z =  t.transform.translation.z;
    }
    catch(const tf2::TransformException & ex){
        return false;
    }
    return true;    
}

void PathPlannerNode::processGoalRequest(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (!is_configured){
        configurePlanner();
    }

    if (!got_map){
        RCLCPP_WARN_STREAM(this->get_logger(), "Goal rejected: no map available yet.");
        publishStatus("Goal rejected: no map available yet");
        return;
    }

    if (is_planning){
        RCLCPP_WARN_STREAM(this->get_logger(), "Goal rejected: planner is busy with a previous request.");
        publishStatus("Goal rejected: planner is busy");
        return;
    }

    if (!get_parameter("read_pose_from_topic").as_bool())
    {
        if (!updatePoseFromTF()){
            RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot determine start pose: TF lookup "
                                << get_parameter("world_frame").as_string() << " <- "
                                << get_parameter("robot_frame").as_string() << " failed.");
            return;
        }
    }

    start_pose_rbs.position = Eigen::Vector3d(start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z);
    start_pose_rbs.orientation = Eigen::Quaterniond(start_pose.pose.orientation.w, start_pose.pose.orientation.x, start_pose.pose.orientation.y, start_pose.pose.orientation.z);

    goal_pose_rbs.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    goal_pose_rbs.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    if (!waypoint_queue.empty()){
        planThroughWaypoints();
        return;
    }

    plan();
}

void PathPlannerNode::addWaypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    waypoint_queue.push_back(msg->pose);
    RCLCPP_INFO_STREAM(this->get_logger(), "Added waypoint " << waypoint_queue.size()
                       << " at (" << msg->pose.position.x << ", " << msg->pose.position.y
                       << ", " << msg->pose.position.z << ")");
    publishWaypointMarkers();
    publishStatus(std::to_string(waypoint_queue.size()) + " waypoint(s) queued");
}

void PathPlannerNode::clearWaypointsCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                             std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    const size_t count = waypoint_queue.size();
    waypoint_queue.clear();
    publishWaypointMarkers();
    response->success = true;
    response->message = "Cleared " + std::to_string(count) + " waypoint(s).";
    RCLCPP_INFO_STREAM(this->get_logger(), response->message);
    publishStatus(response->message);
}

void PathPlannerNode::removeLastWaypointCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (waypoint_queue.empty()){
        response->success = false;
        response->message = "No waypoints queued.";
        return;
    }
    waypoint_queue.pop_back();
    publishWaypointMarkers();
    response->success = true;
    response->message = std::to_string(waypoint_queue.size()) + " waypoint(s) remaining.";
    RCLCPP_INFO_STREAM(this->get_logger(), "Removed last waypoint; " << response->message);
    publishStatus("Removed last waypoint; " + response->message);
}

void PathPlannerNode::editWaypointCallback(const std::shared_ptr<ugv_nav4d_ros2::srv::EditWaypoint::Request> request,
                                           std::shared_ptr<ugv_nav4d_ros2::srv::EditWaypoint::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (request->index == 0 || request->index > waypoint_queue.size()){
        response->success = false;
        response->message = "Waypoint " + std::to_string(request->index) + " does not exist ("
                            + std::to_string(waypoint_queue.size()) + " queued).";
        publishStatus(response->message);
        return;
    }
    const size_t i = request->index - 1;
    if (request->remove){
        waypoint_queue.erase(waypoint_queue.begin() + i);
        response->message = "Deleted waypoint " + std::to_string(request->index) + "; "
                            + std::to_string(waypoint_queue.size()) + " remaining (renumbered).";
    } else {
        waypoint_queue[i] = request->pose;
        response->message = "Replaced waypoint " + std::to_string(request->index) + ".";
    }
    response->success = true;
    publishWaypointMarkers();
    RCLCPP_INFO_STREAM(this->get_logger(), response->message);
    publishStatus(response->message);
}

void PathPlannerNode::publishWaypointMarkers(){
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);

    const std::string frame = get_parameter("world_frame").as_string();
    const auto now = this->get_clock()->now();
    // Queued waypoints are body-frame poses (ground + distToGround); draw the
    // markers back at ground level so they sit on the clicked patch.
    const double dist_to_ground = get_parameter("distToGround").as_double();

    for (size_t i = 0; i < waypoint_queue.size(); ++i){
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = frame;
        arrow.header.stamp = now;
        arrow.ns = "waypoints";
        arrow.id = static_cast<int>(2 * i);
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.pose = waypoint_queue[i];
        arrow.pose.position.z -= dist_to_ground;
        arrow.scale.x = 1.0;
        arrow.scale.y = 0.15;
        arrow.scale.z = 0.15;
        arrow.color.r = 0.0;
        arrow.color.g = 0.8;
        arrow.color.b = 1.0;
        arrow.color.a = 1.0;
        marker_array.markers.push_back(arrow);

        visualization_msgs::msg::Marker text;
        text.header.frame_id = frame;
        text.header.stamp = now;
        text.ns = "waypoint_labels";
        text.id = static_cast<int>(2 * i + 1);
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose = waypoint_queue[i];
        text.pose.position.z += 0.7 - dist_to_ground;
        text.scale.z = 0.5;
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.color.a = 1.0;
        text.text = std::to_string(i + 1);
        marker_array.markers.push_back(text);
    }

    waypoint_marker_publisher->publish(marker_array);
}

void PathPlannerNode::planThroughWaypoints(bool record_mission){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);

    // Target sequence: the queued waypoints in click order, then the final goal.
    std::vector<base::samples::RigidBodyState> targets;
    for (const auto& wp : waypoint_queue){
        base::samples::RigidBodyState rbs;
        rbs.position = Eigen::Vector3d(wp.position.x, wp.position.y, wp.position.z);
        rbs.orientation = Eigen::Quaterniond(wp.orientation.w, wp.orientation.x, wp.orientation.y, wp.orientation.z);
        targets.push_back(rbs);
    }
    targets.push_back(goal_pose_rbs);

    base::Time time;
    time.microseconds = (int64_t)(planner_config.maxTime * 1e6);
    const bool dumpOnError = get_parameter("dumpOnError").as_bool();
    const bool dumpOnSuccess = get_parameter("dumpOnSuccess").as_bool();

    if (record_mission){
        last_mission_start = start_pose_rbs;
        last_mission_goal = goal_pose_rbs;
        has_last_mission_start = true;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Planner state: Planning through "
                       << waypoint_queue.size() << " waypoint(s)");
    publishStatus("Planning through " + std::to_string(waypoint_queue.size()) + " waypoint(s)...");
    is_planning = true;
    struct PlanningFlagGuard {
        std::atomic<bool>& flag;
        ~PlanningFlagGuard() { flag = false; }
    } planning_flag_guard{is_planning};

    std::vector<trajectory_follower::SubTrajectory> combined2D, combined3D;
    base::samples::RigidBodyState segment_start = start_pose_rbs;
    const auto chain_t0 = std::chrono::steady_clock::now();

    for (size_t k = 0; k < targets.size(); ++k){
        std::vector<trajectory_follower::SubTrajectory> trajectory2D, trajectory3D;
        RCLCPP_INFO_STREAM(this->get_logger(), "Leg " << (k + 1) << "/" << targets.size()
                           << ": (" << segment_start.position.transpose()
                           << ") -> (" << targets[k].position.transpose() << ")");

        const ugv_nav4d::Planner::PLANNING_RESULT res =
            planner_ptr->plan(time, segment_start, targets[k], trajectory2D, trajectory3D, dumpOnError, dumpOnSuccess);

        if (res != ugv_nav4d::Planner::FOUND_SOLUTION){
            // Abort the whole mission and clear any stale path: a partial path up to a
            // failed segment must not be executed. The queue is kept so the operator can
            // fix the offending waypoint and retry.
            RCLCPP_ERROR_STREAM(this->get_logger(), "Waypoint leg " << (k + 1) << "/"
                                << targets.size() << " failed: " << planningResultToString(res)
                                << ". Mission aborted; waypoint queue kept for correction.");
            publishStatus(std::string("Waypoint leg ") + std::to_string(k + 1) + "/" +
                          std::to_string(targets.size()) + " failed: " + planningResultToString(res));
            latest_trajectory2D.clear();
            latest_trajectory3D.clear();
            latest_planning_result = res;
            publishPlannedPath({}, false);
            return;
        }

        combined2D.insert(combined2D.end(), trajectory2D.begin(), trajectory2D.end());
        combined3D.insert(combined3D.end(), trajectory3D.begin(), trajectory3D.end());
        segment_start = targets[k];
    }

    latest_trajectory2D = combined2D;
    latest_trajectory3D = combined3D;
    latest_planning_result = ugv_nav4d::Planner::FOUND_SOLUTION;

    const double chain_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - chain_t0).count();
    std::ostringstream chain_msg;
    chain_msg << "FOUND_SOLUTION: " << targets.size() << " leg(s), " << combined3D.size()
              << " trajectory segment(s) (" << std::fixed << std::setprecision(2) << chain_seconds << " s)";
    RCLCPP_INFO_STREAM(this->get_logger(), chain_msg.str());
    publishStatus(chain_msg.str());

    publishPlannedPath(combined3D, true);

    // The queue is deliberately kept after planning: re-sending a goal replans
    // through the same waypoints (e.g. after editing one). It is only emptied
    // by the explicit clear_waypoints service / panel button.
}

void PathPlannerNode::publishStatus(const std::string& status){
    std_msgs::msg::String msg;
    msg.data = status;
    status_publisher->publish(msg);
    if (status_callback) {
        status_callback(status);
    }
}

void PathPlannerNode::saveMapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    if (!got_map || !mls_map_ptr){
        response->success = false;
        response->message = "No MLS map available to save.";
        publishStatus(response->message);
        return;
    }
    const std::string filename = generateTimestampedFilename(".bin");
    if (saveMLSMapAsBin(filename)){
        response->success = true;
        response->message = "MLS map saved to " + filename;
    } else {
        response->success = false;
        response->message = "Failed to save MLS map to " + filename;
    }
    publishStatus(response->message);
}

// Even-odd rule point-in-polygon test in the XY plane.
static bool pointInPolygonXY(double x, double y, const std::vector<geometry_msgs::msg::Point>& poly){
    bool inside = false;
    const size_t n = poly.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++){
        const double xi = poly[i].x, yi = poly[i].y;
        const double xj = poly[j].x, yj = poly[j].y;
        if (((yi > y) != (yj > y)) &&
            (x < (xj - xi) * (y - yi) / (yj - yi) + xi)){
            inside = !inside;
        }
    }
    return inside;
}

void PathPlannerNode::addForbiddenZoneCallback(const ugv_nav4d_ros2::msg::ForbiddenZone::SharedPtr msg){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (msg->vertices.size() < 3){
        RCLCPP_WARN_STREAM(this->get_logger(), "Ignoring forbidden zone with only "
                           << msg->vertices.size() << " vertices (minimum 3).");
        return;
    }
    forbidden_zones.push_back(*msg);
    RCLCPP_INFO_STREAM(this->get_logger(), "Added forbidden zone " << forbidden_zones.size()
                       << " with " << msg->vertices.size() << " vertices.");
    onForbiddenZonesChanged();
    publishStatus(std::to_string(forbidden_zones.size()) + " forbidden zone(s) active");
}

void PathPlannerNode::clearForbiddenZonesCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (forbidden_zones.empty()){
        // Nothing to clear; skip the (expensive) trav map regeneration.
        response->success = true;
        response->message = "No forbidden zones set; nothing to clear.";
        return;
    }
    const size_t count = forbidden_zones.size();
    forbidden_zones.clear();
    onForbiddenZonesChanged();
    response->success = true;
    response->message = "Cleared " + std::to_string(count) + " forbidden zone(s).";
    RCLCPP_INFO_STREAM(this->get_logger(), response->message);
    publishStatus(response->message);
}

void PathPlannerNode::removeLastForbiddenZoneCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (forbidden_zones.empty()){
        response->success = false;
        response->message = "No forbidden zones set.";
        return;
    }
    forbidden_zones.pop_back();
    onForbiddenZonesChanged();
    response->success = true;
    response->message = std::to_string(forbidden_zones.size()) + " forbidden zone(s) remaining.";
    RCLCPP_INFO_STREAM(this->get_logger(), "Removed last forbidden zone; " << response->message);
    publishStatus("Removed last forbidden zone; " + response->message);
}

void PathPlannerNode::onForbiddenZonesChanged(){
    publishForbiddenZoneMarkers();
    // Zones are baked into the trav map during generation, so a change requires a
    // rebuild. Clearing a zone has no cheaper path anyway (original node types are
    // not stored). Also refreshes the trav map in RViz.
    if (is_configured && got_map){
        configurePlanner();
        publishTravMap();
    }
}

void PathPlannerNode::applyForbiddenZones(){
    if (forbidden_zones.empty() || !traversability_generator_ptr){
        return;
    }
    // The map reference is const, but it stores non-const TravGenNode pointers,
    // so the nodes themselves can be re-typed.
    const auto& trav_map_3d = traversability_generator_ptr->getTraversabilityMap();
    size_t marked = 0;

    for (const auto& zone : forbidden_zones)
    {
        // Flood-fill the connected surface the polygon was drawn on: seed at the
        // patches nearest to the clicked vertices, then grow along the trav
        // graph's neighbor connections while staying inside the polygon. Other
        // storeys are separate connected components, so they are never touched --
        // no height band or margin needed.
        std::deque<traversability_generator3d::TravGenNode*> queue;
        std::unordered_set<traversability_generator3d::TravGenNode*> visited;

        for (const auto& v : zone.vertices)
        {
            maps::grid::Index idx;
            if (!trav_map_3d.toGrid(Eigen::Vector3d(v.x, v.y, v.z), idx)){
                continue;
            }
            traversability_generator3d::TravGenNode* seed = nullptr;
            double best_dz = std::numeric_limits<double>::max();
            for (traversability_generator3d::TravGenNode* n : trav_map_3d.at(idx))
            {
                const double dz = std::abs(n->getHeight() - v.z);
                if (dz < best_dz){
                    best_dz = dz;
                    seed = n;
                }
            }
            if (seed && visited.insert(seed).second){
                queue.push_back(seed);
            }
        }

        if (queue.empty()){
            RCLCPP_WARN_STREAM(this->get_logger(), "Forbidden zone with " << zone.vertices.size()
                               << " vertices matched no traversability patches; zone has no effect.");
            continue;
        }

        while (!queue.empty())
        {
            traversability_generator3d::TravGenNode* node = queue.front();
            queue.pop_front();

            // Same marking travgen uses for real obstacles, so all planner
            // checks (goal validity, expansions, obstacle checks) respect it.
            node->setType(maps::grid::TraversabilityNodeBase::OBSTACLE);
            node->getUserData().nodeType = traversability_generator3d::NodeType::OBSTACLE;
            ++marked;

            for (maps::grid::TraversabilityNodeBase* nb : node->getConnections())
            {
                auto* neighbor = static_cast<traversability_generator3d::TravGenNode*>(nb);
                if (visited.count(neighbor)){
                    continue;
                }
                Eigen::Vector3d position;
                trav_map_3d.fromGrid(neighbor->getIndex(), position, neighbor->getHeight(), false);
                if (!pointInPolygonXY(position.x(), position.y(), zone.vertices)){
                    continue;
                }
                visited.insert(neighbor);
                queue.push_back(neighbor);
            }
        }
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Applied " << forbidden_zones.size()
                       << " forbidden zone(s): " << marked << " trav node(s) marked as obstacle.");
}

void PathPlannerNode::deleteForbiddenZoneCallback(const std::shared_ptr<ugv_nav4d_ros2::srv::DeleteForbiddenZone::Request> request,
                                                  std::shared_ptr<ugv_nav4d_ros2::srv::DeleteForbiddenZone::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (request->index == 0 || request->index > forbidden_zones.size()){
        response->success = false;
        response->message = "Forbidden zone " + std::to_string(request->index) + " does not exist ("
                            + std::to_string(forbidden_zones.size()) + " active).";
        publishStatus(response->message);
        return;
    }
    forbidden_zones.erase(forbidden_zones.begin() + (request->index - 1));
    onForbiddenZonesChanged();
    response->success = true;
    response->message = "Deleted forbidden zone " + std::to_string(request->index) + "; "
                        + std::to_string(forbidden_zones.size()) + " remaining (renumbered).";
    RCLCPP_INFO_STREAM(this->get_logger(), response->message);
    publishStatus(response->message);
}

void PathPlannerNode::planReturnCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (!has_last_mission_start){
        response->success = false;
        response->message = "No previous mission start recorded; plan a mission first.";
        publishStatus(response->message);
        return;
    }
    if (!is_configured){
        configurePlanner();
    }
    if (!got_map){
        response->success = false;
        response->message = "Cannot plan return: no map available.";
        publishStatus(response->message);
        return;
    }
    if (is_planning){
        response->success = false;
        response->message = "Cannot plan return: planner is busy.";
        publishStatus(response->message);
        return;
    }

    // Build the complete reverse mission, beginning at the outward mission's
    // final goal. Using the live robot pose here could incorrectly omit the
    // final-goal -> last-waypoint leg (for example, while the robot is still at
    // the last waypoint).
    start_pose_rbs = last_mission_goal;
    goal_pose_rbs = last_mission_start;

    const bool face_forward = get_parameter("return_face_forward").as_bool();
    const Eigen::Quaterniond half_turn(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
    if (face_forward){
        // Keep the real orientation at the final goal as the planning start, so
        // the path includes the turn-around instead of assuming it already
        // happened. Reverse the heading requested at every return target.
        goal_pose_rbs.orientation = (goal_pose_rbs.orientation * half_turn).normalized();
    }

    publishStatus(face_forward
        ? "Planning forward-facing return to the last mission start..."
        : "Planning return to the last mission start (reverse motion allowed)...");
    is_return_plan = true;
    struct ReturnPlanFlagGuard {
        bool& flag;
        ~ReturnPlanFlagGuard() { flag = false; }
    } return_plan_flag_guard{is_return_plan};

    if (waypoint_queue.empty()){
        plan(false);
    } else {
        // Visit the waypoints in reverse order on the way back. The queue itself
        // is restored afterwards (it is only emptied by clear_waypoints).
        const auto outward_waypoints = waypoint_queue;
        std::reverse(waypoint_queue.begin(), waypoint_queue.end());
        if (face_forward){
            for (auto& waypoint : waypoint_queue){
                Eigen::Quaterniond orientation(
                    waypoint.orientation.w, waypoint.orientation.x,
                    waypoint.orientation.y, waypoint.orientation.z);
                orientation = (orientation * half_turn).normalized();
                waypoint.orientation.x = orientation.x();
                waypoint.orientation.y = orientation.y();
                waypoint.orientation.z = orientation.z();
                waypoint.orientation.w = orientation.w();
            }
        }
        planThroughWaypoints(false);
        waypoint_queue = outward_waypoints;
    }

    response->success = (latest_planning_result == ugv_nav4d::Planner::FOUND_SOLUTION);
    response->message = response->success
        ? "Return path ready - review and Execute."
        : std::string("Return planning failed: ") + planningResultToString(latest_planning_result);
}

void PathPlannerNode::setReturnForwardCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response){
    const auto result = set_parameter(rclcpp::Parameter("return_face_forward", request->data));
    response->success = result.successful;
    response->message = result.successful
        ? (request->data ? "Return mode: turn around and prefer forward driving."
                         : "Return mode: preserve headings; reverse driving allowed.")
        : "Could not change return mode: " + result.reason;
    publishStatus(response->message);
}

void PathPlannerNode::publishForbiddenZoneMarkers(){
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);

    const std::string frame = get_parameter("world_frame").as_string();
    const auto now = this->get_clock()->now();

    for (size_t i = 0; i < forbidden_zones.size(); ++i){
        const auto& zone = forbidden_zones[i];
        const size_t n_verts = zone.vertices.size();
        if (n_verts < 3){
            continue;
        }

        // Closed outline on the ground.
        visualization_msgs::msg::Marker outline;
        outline.header.frame_id = frame;
        outline.header.stamp = now;
        outline.ns = "forbidden_zones";
        outline.id = static_cast<int>(2 * i);
        outline.type = visualization_msgs::msg::Marker::LINE_STRIP;
        outline.action = visualization_msgs::msg::Marker::ADD;
        outline.pose.orientation.w = 1.0;
        outline.scale.x = 0.1;
        outline.color.r = 1.0;
        outline.color.g = 0.1;
        outline.color.b = 0.1;
        outline.color.a = 1.0;
        for (size_t k = 0; k <= n_verts; ++k){
            outline.points.push_back(zone.vertices[k % n_verts]);
        }
        marker_array.markers.push_back(outline);

        // Translucent vertical wall along the edges (works for concave polygons,
        // no triangulation of the interior needed).
        visualization_msgs::msg::Marker wall;
        wall.header.frame_id = frame;
        wall.header.stamp = now;
        wall.ns = "forbidden_zone_walls";
        wall.id = static_cast<int>(2 * i + 1);
        wall.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        wall.action = visualization_msgs::msg::Marker::ADD;
        wall.pose.orientation.w = 1.0;
        wall.scale.x = 1.0;
        wall.scale.y = 1.0;
        wall.scale.z = 1.0;
        wall.color.r = 1.0;
        wall.color.g = 0.1;
        wall.color.b = 0.1;
        wall.color.a = 0.3;
        const double wall_height = 2.0;
        for (size_t k = 0; k < n_verts; ++k){
            geometry_msgs::msg::Point a = zone.vertices[k];
            geometry_msgs::msg::Point b = zone.vertices[(k + 1) % n_verts];
            geometry_msgs::msg::Point a_top = a;
            geometry_msgs::msg::Point b_top = b;
            a_top.z += wall_height;
            b_top.z += wall_height;
            // two triangles per edge: (a, b, b_top) and (a, b_top, a_top)
            wall.points.push_back(a);
            wall.points.push_back(b);
            wall.points.push_back(b_top);
            wall.points.push_back(a);
            wall.points.push_back(b_top);
            wall.points.push_back(a_top);
        }
        marker_array.markers.push_back(wall);

        // Zone number above the wall, for delete-by-number.
        geometry_msgs::msg::Point centroid;
        for (const auto& v : zone.vertices){
            centroid.x += v.x;
            centroid.y += v.y;
            centroid.z += v.z;
        }
        centroid.x /= static_cast<double>(n_verts);
        centroid.y /= static_cast<double>(n_verts);
        centroid.z /= static_cast<double>(n_verts);

        visualization_msgs::msg::Marker label;
        label.header.frame_id = frame;
        label.header.stamp = now;
        label.ns = "forbidden_zone_labels";
        label.id = static_cast<int>(i);
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::msg::Marker::ADD;
        label.pose.position = centroid;
        label.pose.position.z += 2.4;
        label.pose.orientation.w = 1.0;
        label.scale.z = 0.8;
        label.color.r = 1.0;
        label.color.g = 0.3;
        label.color.b = 0.3;
        label.color.a = 1.0;
        label.text = std::to_string(i + 1);
        marker_array.markers.push_back(label);
    }

    forbidden_zone_marker_publisher->publish(marker_array);
}

const char* PathPlannerNode::planningResultToString(ugv_nav4d::Planner::PLANNING_RESULT res){
    switch(res)
    {
        case ugv_nav4d::Planner::FOUND_SOLUTION: return "FOUND_SOLUTION";
        case ugv_nav4d::Planner::GOAL_INVALID:   return "GOAL_INVALID";
        case ugv_nav4d::Planner::START_INVALID:  return "START_INVALID";
        case ugv_nav4d::Planner::INTERNAL_ERROR: return "INTERNAL_ERROR";
        case ugv_nav4d::Planner::NO_SOLUTION:    return "NO_SOLUTION";
        case ugv_nav4d::Planner::NO_MAP:         return "NO_MAP";
    }
    return "UNKNOWN";
}

void PathPlannerNode::readStartPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg){

    start_pose.pose.position.x = msg->pose.position.x;
    start_pose.pose.position.y = msg->pose.position.y;
    start_pose.pose.position.z = msg->pose.position.z;

    start_pose.pose.orientation.w = msg->pose.orientation.w;
    start_pose.pose.orientation.x = msg->pose.orientation.x;
    start_pose.pose.orientation.y = msg->pose.orientation.y;
    start_pose.pose.orientation.z = msg->pose.orientation.z;
}

void PathPlannerNode::initializeMLSMap(){
    // (Re)create the MLS map from the current parameters. Called at startup and whenever
    // grid_resolution changes. Recreating the map discards any previously accumulated data.
    initial_patch_added = false;
    current_grid_resolution = get_parameter("grid_resolution").as_double();

    if (get_parameter("load_mls_from_file").as_bool()){
        const std::string mls_file_path = get_parameter("mls_file_path").as_string();
        const std::string mls_file_type = get_parameter("mls_file_type").as_string();

        if (mls_file_type == "ply"){
            if (loadPlyAsMLS(mls_file_path)){
                got_map = true;
                RCLCPP_INFO_STREAM(this->get_logger(), "Loaded MLS from PLY '" << mls_file_path << "' ("
                                   << mls_map_ptr->getNumCells().transpose() << " cells @ "
                                   << current_grid_resolution << " m).");
            }
        }
        else if (mls_file_type == "bin"){
            // A .bin map has its resolution baked in; grid_resolution cannot change it.
            if(loadMLSMapFromBin(mls_file_path)){
                got_map = true;
                RCLCPP_INFO_STREAM(this->get_logger(), "Loaded MLS from BIN '" << mls_file_path << "'.");
                if (mls_map_ptr){
                    const double loaded_res = mls_map_ptr->getResolution().x();
                    if (std::abs(loaded_res - current_grid_resolution) > 1e-9){
                        RCLCPP_WARN_STREAM(this->get_logger(), "grid_resolution ("
                            << current_grid_resolution << ") ignored for .bin map; using the map's baked-in resolution ("
                            << loaded_res << ").");
                        // Track the actual resolution so we don't repeatedly try to recreate the map.
                        current_grid_resolution = loaded_res;
                    }
                }
            }
        }
        else{
            throw std::runtime_error("Invalid MLS File Type: "+ mls_file_type);
        }
    }
    else{
        const double mls_res = current_grid_resolution;
        const double dist_max_x = get_parameter("dist_max_x").as_int();
        const double dist_max_y = get_parameter("dist_max_y").as_int();
        const double dist_min_x = get_parameter("dist_min_x").as_int();
        const double dist_min_y = get_parameter("dist_min_y").as_int();
        const double grid_size_x = (dist_max_x - dist_min_x)/mls_res;
        const double grid_size_y = (dist_max_y - dist_min_y)/mls_res;

        maps::grid::MLSConfig cfg;
        cfg.gapSize = get_parameter("mls_gap_size").as_double();
        const maps::grid::Vector2ui numCells(grid_size_x, grid_size_y);
        mls_map_ptr = std::make_shared<maps::grid::MLSMapSloped>(numCells, maps::grid::Vector2d(mls_res, mls_res), cfg);
        mls_map_ptr->translate(Eigen::Vector3d(mls_min_x, mls_min_y, 0));
        // Fresh empty grid: any point cloud data accumulated at the old resolution is gone.
        got_map = false;
    }
}

bool PathPlannerNode::loadPlyAsMLS(const std::string& path){
    std::ifstream fileIn(path);
    if(path.find(".ply") != std::string::npos)
    {
        cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PLYReader plyReader;
        if(plyReader.read(path, *cloud) >= 0)
        {
            pcl::PointXYZ min, max; 
            pcl::getMinMax3D (*cloud, min, max); 

            const double size_x = max.x - min.x;
            const double size_y = max.y - min.y;

            mls_min_x = min.x;
            mls_min_y = min.y;

            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            box_filter.setInputCloud(cloud);
            box_filter.filter(*cloud_filtered);

            // Optional voxel-grid downsampling of the raw cloud (0 = disabled). Reduces load
            // time/memory for dense PLYs; a leaf >= grid_resolution loses no map fidelity since
            // the MLS already bins at grid_resolution.
            const double leaf = get_parameter("ply_downsample_leaf_size").as_double();
            if (leaf > 0.0){
                const size_t before = cloud_filtered->size();
                pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
                voxel_grid.setInputCloud(cloud_filtered);
                voxel_grid.setLeafSize(leaf, leaf, leaf);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
                voxel_grid.filter(*cloud_downsampled);
                cloud_filtered = cloud_downsampled;
                RCLCPP_INFO_STREAM(this->get_logger(), "Downsampled PLY cloud from " << before
                                   << " to " << cloud_filtered->size() << " points (leaf " << leaf << " m).");
            }

            const double mls_res = get_parameter("grid_resolution").as_double();
            
            maps::grid::MLSConfig cfg;
            cfg.gapSize = get_parameter("mls_gap_size").as_double();
            const maps::grid::Vector2ui numCells(size_x / mls_res + 1, size_y / mls_res + 1);

            mls_map_ptr = std::make_shared<maps::grid::MLSMapSloped>(numCells, maps::grid::Vector2d(mls_res, mls_res), cfg);
            mls_map_ptr->translate(Eigen::Vector3d(mls_min_x, mls_min_y, 0));
            mls_map_ptr->mergePointCloud(*cloud_filtered, base::Transform3d::Identity());
        }
        return true;
    }
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to load MLS from '" << path << "': unknown format (expected .ply).");
    return false;
}

bool PathPlannerNode::generateMLS(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*latest_pointcloud, *cloud);

    std::string world_frame = get_parameter("world_frame").as_string();
    base::Transform3d cloud2MLS;

    if (latest_pointcloud->header.frame_id != world_frame){
        try{
            geometry_msgs::msg::TransformStamped t = tf_buffer_ptr->lookupTransform(world_frame, latest_pointcloud->header.frame_id, tf2::TimePointZero);
            cloud2MLS.translation() << t.transform.translation.x, t.transform.translation.y, t.transform.translation.z;
            Eigen::Quaterniond quat(t.transform.rotation.w, 
                                    t.transform.rotation.x, 
                                    t.transform.rotation.y, 
                                    t.transform.rotation.z);
            cloud2MLS.linear() = quat.toRotationMatrix();
        }
        catch(const tf2::TransformException & ex){
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to transform " << latest_pointcloud->header.frame_id << " to "  << world_frame);
            return false;
        }
    }
    else{
        cloud2MLS.translation() << 0,0,0;
        Eigen::Quaterniond quat(1,0,0,0);
        cloud2MLS.linear() = quat.toRotationMatrix();
    }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    box_filter.setInputCloud(cloud);
    box_filter.filter(*cloud_filtered);

    mls_map_ptr->mergePointCloud(*cloud_filtered, cloud2MLS);
    return true;
}

// Function to save the MLS map as a binary file
bool PathPlannerNode::saveMLSMapAsBin(const std::string& filename = "") {
    std::string fileToUse;

    // Check if filename is provided, if not generate one
    if (filename.empty()) {
        fileToUse = generateTimestampedFilename(".bin");
    } else {
        fileToUse = filename;
    }

    // Open a binary file for output
    std::ofstream binFile(fileToUse, std::ios::binary);
    if (!binFile) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot open file for writing: " << fileToUse);
        return false;
    }

    // Create a binary archive. Lock while serializing the map, since the save action runs on
    // its own thread and the MLS map may be rebuilt concurrently by the executor thread.
    {
        std::lock_guard<std::recursive_mutex> lock(planner_mutex);
        boost::archive::binary_oarchive archive(binFile);
        archive << *mls_map_ptr;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "MLS Map saved to " << fileToUse);
    return true;
}

bool PathPlannerNode::loadMLSMapFromBin(const std::string& filename){
    if (filename.empty()) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Failed to load MLS Map from empty file: " << filename);
        return false;
    }

    // Open the binary file in input mode
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Cannot open MLS map file for reading: " << filename);
        return false;
    }

    try {
        // Load the file contents into the stream and deserialize
        boost::archive::binary_iarchive ia(file);
        mls_map_ptr = std::make_shared<maps::grid::MLSMapSloped>();
        ia >> *mls_map_ptr;  // Deserialize directly into the object pointed by the shared_ptr

        RCLCPP_INFO_STREAM(this->get_logger(), "Loaded MLS Map from " << filename);
        return true;
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Error loading MLS Map: " << e.what());
        return false;
    }
}

rclcpp_action::GoalResponse PathPlannerNode::actionSaveMap(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SaveMLSMap::Goal> goal)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Save map action: filename '" << goal->filename << "' (empty = timestamped).");
    (void)uuid;  // Suppress unused variable warning
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;  // Accept the goal
}

rclcpp_action::CancelResponse PathPlannerNode::actionCancelSaveMap(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SaveMLSMap>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Canceling save map request");
    return rclcpp_action::CancelResponse::ACCEPT;  // Accept the cancel request
}

void PathPlannerNode::actionSaveMapAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SaveMLSMap>> goal_handle)
{
    std::thread{
        [this, goal_handle]() {
            const auto filename = goal_handle->get_goal()->filename; // Get the filename from the goal

            saveMLSMapAsBin(filename); // Call save function with the filename

            // Mark the goal as succeeded
            const auto result = std::make_shared<SaveMLSMap::Result>();
            result->success = true;  // Set the success flag
            goal_handle->succeed(result);
        }
    }.detach();
}

void PathPlannerNode::plan(bool record_mission){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);

    std::vector<trajectory_follower::SubTrajectory> trajectory2D, trajectory3D;
    base::Time time;
    time.microseconds = (int64_t)(planner_config.maxTime * 1e6);

    bool dumpOnError = get_parameter("dumpOnError").as_bool();
    bool dumpOnSuccess = get_parameter("dumpOnSuccess").as_bool();

    // RAII guard: always clears is_planning, even if planner_ptr->plan() throws.
    is_planning = true;
    struct PlanningFlagGuard {
        std::atomic<bool>& flag;
        ~PlanningFlagGuard() { flag = false; }
    } planning_flag_guard{is_planning};

    if (record_mission){
        last_mission_start = start_pose_rbs;
        last_mission_goal = goal_pose_rbs;
        has_last_mission_start = true;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Planning: start (" << start_pose_rbs.position.transpose()
                       << ") -> goal (" << goal_pose_rbs.position.transpose()
                       << "), time budget " << planner_config.maxTime << " s");
    publishStatus("Planning...");

    const auto plan_t0 = std::chrono::steady_clock::now();
    ugv_nav4d::Planner::PLANNING_RESULT res = planner_ptr->plan(time, start_pose_rbs, goal_pose_rbs, trajectory2D, trajectory3D, dumpOnError, dumpOnSuccess);
    const double plan_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - plan_t0).count();

    latest_trajectory2D = trajectory2D;
    latest_trajectory3D = trajectory3D;
    latest_planning_result = res;

    std::ostringstream result_msg;
    result_msg << planningResultToString(res);
    if (res == ugv_nav4d::Planner::FOUND_SOLUTION) {
        result_msg << ": " << trajectory3D.size() << " trajectory segment(s)";
    }
    result_msg << " (" << std::fixed << std::setprecision(2) << plan_seconds << " s)";
    if (res == ugv_nav4d::Planner::FOUND_SOLUTION) {
        RCLCPP_INFO_STREAM(this->get_logger(), result_msg.str());
    } else {
        RCLCPP_ERROR_STREAM(this->get_logger(), result_msg.str());
    }
    publishStatus(result_msg.str());

    publishPlannedPath(trajectory3D, res == ugv_nav4d::Planner::FOUND_SOLUTION);
}

void PathPlannerNode::publishPlannedPath(const std::vector<trajectory_follower::SubTrajectory>& trajectory3D, bool found_solution)
{
    ugv_nav4d_ros2::msg::LabeledPathArray labeled_path_message;
    visualization_msgs::msg::MarkerArray colored_path_message;
    auto now = this->get_clock()->now();
    const std::string world_frame = get_parameter("world_frame").as_string();

    visualization_msgs::msg::Marker clear_colored_path;
    clear_colored_path.action = visualization_msgs::msg::Marker::DELETEALL;
    colored_path_message.markers.push_back(clear_colored_path);

    nav_msgs::msg::Path path;
    path.header.frame_id = world_frame;

    nav_msgs::msg::Path path_segment;
    std::string label_last;
    std::string label;
    bool first_segment = true;

    if (found_solution) {
        for (size_t seg_idx = 0; seg_idx < trajectory3D.size(); ++seg_idx) {
            auto& trajectory = trajectory3D[seg_idx];

            // Assign label based on current segment
            if (trajectory.driveMode == trajectory_follower::DriveMode::ModeAckermann && trajectory.speed > 0) {
                label = "Forward";
            } else if (trajectory.driveMode == trajectory_follower::DriveMode::ModeAckermann && trajectory.speed < 0) {
                label = "Backward";
            } else if (trajectory.driveMode == trajectory_follower::DriveMode::ModeTurnOnTheSpot) {
                label = "PointTurn";
            } else if (trajectory.driveMode == trajectory_follower::DriveMode::ModeSideways) {
                label = "Lateral";
            } else {
                // Unknown drive mode: skip this segment instead of throwing (an uncaught
                // exception here would abort the whole process).
                RCLCPP_WARN_STREAM(this->get_logger(), "Skipping segment with invalid DriveMode: "
                                   << std::to_string(trajectory.driveMode));
                continue;
            }

            // If this is a new segment (first or label changed), start a new path_segment
            if (first_segment || label != label_last) {
                if (!first_segment && !path_segment.poses.empty()) {
                    labeled_path_message.paths.push_back(path_segment);
                    labeled_path_message.labels.push_back(label_last);
                }

                path_segment = nav_msgs::msg::Path();
                path_segment.header.frame_id = world_frame;
                label_last = label;
                first_segment = false;
            }

            // Sample spline and add poses to current segment
            const double stepDist = get_parameter("spline_sampling_resolution").as_double();
            std::vector<double> parameters;
            const std::vector<base::geometry::Spline3::vector_t> points = trajectory.posSpline.sample(stepDist, &parameters);
            if (parameters.size() != points.size()) {
                // Should never happen; log instead of relying on assert (compiled out in release).
                RCLCPP_WARN_STREAM(this->get_logger(), "Spline sample size mismatch: "
                                   << parameters.size() << " params vs " << points.size() << " points.");
            }

            // Draw two coincident lines: a wide, translucent mission-color
            // underlay (blue outward / orange return), and a narrow motion-color
            // overlay (green forward / red backward). This keeps both pieces of
            // operator information visible at the same time.
            if (!points.empty()) {
                visualization_msgs::msg::Marker route_line;
                route_line.header.frame_id = world_frame;
                route_line.header.stamp = now;
                route_line.ns = is_return_plan ? "return_route" : "outward_route";
                route_line.id = static_cast<int>(2 * seg_idx);
                route_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
                route_line.action = visualization_msgs::msg::Marker::ADD;
                route_line.pose.orientation.w = 1.0;
                route_line.scale.x = 0.22;
                route_line.color.a = 0.65;
                if (is_return_plan) {
                    route_line.color.r = 1.0;
                    route_line.color.g = 0.45;
                    route_line.color.b = 0.05;
                } else {
                    route_line.color.r = 0.1;
                    route_line.color.g = 0.35;
                    route_line.color.b = 1.0;
                }

                visualization_msgs::msg::Marker motion_line = route_line;
                motion_line.ns = "motion_direction";
                motion_line.id = static_cast<int>(2 * seg_idx + 1);
                motion_line.scale.x = 0.09;
                motion_line.color.a = 1.0;
                if (label == "Forward") {
                    motion_line.color.r = 0.1;
                    motion_line.color.g = 1.0;
                    motion_line.color.b = 0.1;
                } else if (label == "Backward") {
                    motion_line.color.r = 1.0;
                    motion_line.color.g = 0.05;
                    motion_line.color.b = 0.05;
                } else if (label == "PointTurn") {
                    motion_line.color.r = 1.0;
                    motion_line.color.g = 0.9;
                    motion_line.color.b = 0.0;
                } else {
                    motion_line.color.r = 0.0;
                    motion_line.color.g = 0.9;
                    motion_line.color.b = 1.0;
                }

                for (const auto& point : points) {
                    geometry_msgs::msg::Point marker_point;
                    marker_point.x = point.x();
                    marker_point.y = point.y();
                    marker_point.z = point.z() + 0.05;
                    route_line.points.push_back(marker_point);
                    marker_point.z += 0.02;
                    motion_line.points.push_back(marker_point);
                }
                colored_path_message.markers.push_back(route_line);
                colored_path_message.markers.push_back(motion_line);
            }

            for (size_t i = 0; i < parameters.size(); ++i) {
                const double param = parameters[i];

                double yaw_angle = 0;
                base::Vector3d point, tangent;

                if (trajectory.driveMode == trajectory_follower::DriveMode::ModeTurnOnTheSpot) {
                    point = trajectory.posSpline.getPoint(param);
                    yaw_angle = trajectory.goalPose.orientation;
                } else {
                    std::tie(point, tangent) = trajectory.posSpline.getPointAndTangent(param);
                    if (trajectory.speed < 0) {
                        yaw_angle = std::atan2(-tangent.y(), -tangent.x());
                    } else {
                        yaw_angle = std::atan2(tangent.y(), tangent.x());
                    }
                }

                Eigen::Quaterniond yaw(Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ()));

                geometry_msgs::msg::PoseStamped tempPoint;
                tempPoint.pose.position.x = point.x();
                tempPoint.pose.position.y = point.y();
                tempPoint.pose.position.z = point.z();
                tempPoint.pose.orientation.x = yaw.x();
                tempPoint.pose.orientation.y = yaw.y();
                tempPoint.pose.orientation.z = yaw.z();
                tempPoint.pose.orientation.w = yaw.w();
                tempPoint.header.stamp = now;
                tempPoint.header.frame_id = world_frame;

                path.poses.push_back(tempPoint);
                path_segment.poses.push_back(tempPoint);
            }

            // Extension logic
            if (extend_trajectory && trajectory.driveMode != trajectory_follower::DriveMode::ModeTurnOnTheSpot) {
                bool add_extension_point = false;
                if (seg_idx + 1 < trajectory3D.size()) {
                    const auto& next_traj = trajectory3D[seg_idx + 1];
                    bool curr_fwd = trajectory.speed > 0;
                    bool curr_bwd = trajectory.speed < 0;
                    bool next_fwd = next_traj.speed > 0;
                    bool next_bwd = next_traj.speed < 0;

                    if ((curr_fwd && next_bwd) || (curr_bwd && next_fwd)) {
                        add_extension_point = true;
                    }
                } else if (seg_idx + 1 == trajectory3D.size()) {
                    add_extension_point = true;
                }

                if (add_extension_point && !parameters.empty()) {
                    double last_param = parameters.back();
                    base::Vector3d end_point, end_tangent;
                    std::tie(end_point, end_tangent) = trajectory.posSpline.getPointAndTangent(last_param);

                    Eigen::Vector3d direction = end_tangent.normalized();

                    base::Vector3d ext_point_half = end_point + direction * (extension_distance * 0.5);
                    base::Vector3d ext_point_full = end_point + direction * extension_distance;

                    geometry_msgs::msg::PoseStamped ext_pose_half;
                    ext_pose_half.header.stamp = now;
                    ext_pose_half.header.frame_id = get_parameter("world_frame").as_string();
                    ext_pose_half.pose.position.x = ext_point_half.x();
                    ext_pose_half.pose.position.y = ext_point_half.y();
                    ext_pose_half.pose.position.z = ext_point_half.z();

                    geometry_msgs::msg::PoseStamped ext_pose_full;
                    ext_pose_full.header.stamp = now;
                    ext_pose_full.header.frame_id = get_parameter("world_frame").as_string();
                    ext_pose_full.pose.position.x = ext_point_full.x();
                    ext_pose_full.pose.position.y = ext_point_full.y();
                    ext_pose_full.pose.position.z = ext_point_full.z();

                    if (trajectory.speed < 0) {
                        direction = -direction;
                    }

                    double yaw_angle_half = std::atan2(direction.y(), direction.x());
                    Eigen::Quaterniond yaw_half(Eigen::AngleAxisd(yaw_angle_half, Eigen::Vector3d::UnitZ()));

                    ext_pose_half.pose.orientation.x = yaw_half.x();
                    ext_pose_half.pose.orientation.y = yaw_half.y();
                    ext_pose_half.pose.orientation.z = yaw_half.z();
                    ext_pose_half.pose.orientation.w = yaw_half.w();

                    ext_pose_full.pose.orientation.x = yaw_half.x();
                    ext_pose_full.pose.orientation.y = yaw_half.y();
                    ext_pose_full.pose.orientation.z = yaw_half.z();
                    ext_pose_full.pose.orientation.w = yaw_half.w();

                    path.poses.push_back(ext_pose_half);
                    path.poses.push_back(ext_pose_full);

                    path_segment.poses.push_back(ext_pose_half);
                    path_segment.poses.push_back(ext_pose_full);
                }
            }
        }

        // Final push for the last segment
        if (!path_segment.poses.empty()) {
            labeled_path_message.paths.push_back(path_segment);
            labeled_path_message.labels.push_back(label_last);
        }
    }
    else {
        RCLCPP_WARN_STREAM(this->get_logger(), "Planning did not succeed; publishing empty path to clear any stale trajectory.");
    }

    // Publish unconditionally: on failure this clears the previously published path
    // instead of leaving a stale trajectory on the topic.
    path.header.stamp = now;
    combined_path_publisher->publish(path);
    labeled_path_publisher->publish(labeled_path_message);
    colored_path_publisher->publish(colored_path_message);

    // Hold the path for the execution gate. Anything that plans (or fails to)
    // replaces the pending path, so Execute always sends what RViz shows.
    pending_labeled_path = labeled_path_message;
    has_pending_path = found_solution && !labeled_path_message.paths.empty();
    if (has_pending_path) {
        publishStatus("Path ready (" + std::to_string(pending_labeled_path.paths.size()) +
                      " segment(s)) - review in RViz, then Execute to send to the follower");
    }
}

void PathPlannerNode::executePathCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                          std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (!has_pending_path){
        response->success = false;
        response->message = "No planned path pending; plan first.";
        publishStatus(response->message);
        return;
    }
    execute_path_publisher->publish(pending_labeled_path);
    response->success = true;
    response->message = "Path sent to follower (" +
                        std::to_string(pending_labeled_path.paths.size()) + " segment(s)).";
    RCLCPP_INFO_STREAM(this->get_logger(), response->message);
    publishStatus(response->message);
}

void PathPlannerNode::discardPathCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                          std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (!has_pending_path){
        response->success = false;
        response->message = "No planned path pending.";
        return;
    }
    has_pending_path = false;
    pending_labeled_path = ugv_nav4d_ros2::msg::LabeledPathArray();
    // Clear the preview in RViz as well.
    publishPlannedPath({}, false);
    response->success = true;
    response->message = "Pending path discarded.";
    RCLCPP_INFO_STREAM(this->get_logger(), response->message);
    publishStatus(response->message);
}

void PathPlannerNode::declareParameters(){

    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.read_only = true;

    declare_parameter("read_pose_from_topic", false, param_desc);
    declare_parameter("load_mls_from_file", false, param_desc);
    declare_parameter("mls_file_type", "ply", param_desc);
    declare_parameter("mls_file_path", "default_value", param_desc);
    declare_parameter("robot_frame", "robot", param_desc);
    declare_parameter("world_frame", "map", param_desc);
    declare_parameter("mls_gap_size", 0.1, param_desc);
    declare_parameter("ply_downsample_leaf_size", 0.0, param_desc); // 0 = no downsampling
    declare_parameter("dist_max_x", 50, param_desc);
    declare_parameter("dist_min_x", -50, param_desc);
    declare_parameter("dist_max_y", 50, param_desc);
    declare_parameter("dist_min_y", -50, param_desc);
    // Height (Z) crop bounds are doubles to allow sub-meter height filtering of the input cloud.
    declare_parameter("dist_max_z", 50.0, param_desc);
    declare_parameter("dist_min_z", -50.0, param_desc);
    
    declare_parameter("dumpOnError", false);
    declare_parameter("dumpOnSuccess", false);
    declare_parameter("return_face_forward", true);
    declare_parameter("initialPatchRadius", 3.0);
    declare_parameter("grid_resolution", 0.3);
    declare_parameter("maxMotionCurveLength", 100.0);
    declare_parameter("minTurningRadius", 1.0);
    declare_parameter("multiplierBackward", 3.0);
    declare_parameter("multiplierBackwardTurn", 4.0);
    declare_parameter("multiplierForward", 1.0);
    declare_parameter("multiplierForwardTurn", 2.0);
    declare_parameter("multiplierLateral", 4.0);
    declare_parameter("multiplierLateralCurve", 4.0);
    declare_parameter("multiplierPointTurn", 3.0);
    declare_parameter("rotationSpeed", 1.0);
    declare_parameter("searchProgressSteps", 0.1);
    declare_parameter("searchRadius", 1.0);
    declare_parameter("translationSpeed", 1.0);
    declare_parameter("spline_sampling_resolution", 0.05);
    declare_parameter("remove_goal_offset", false);
    declare_parameter("curvaturePenaltyWeight", 0.0);
    declare_parameter("angularCostWeight", 1.0);

    declare_parameter("epsilonSteps", 2);
    declare_parameter("initialEpsilon", 64);
    declare_parameter("numThreads", 8);
    declare_parameter("usePathStatistics", false);
    declare_parameter("searchUntilFirstSolution", false);
    declare_parameter("corridorWidth", 3.5);
    declare_parameter("maxTime", 5.0);
    declare_parameter("goalOrientationMargin", 0.0);
    declare_parameter("goalDistanceMargin", 0.0);
    declare_parameter("useReedsSheppFinalPath", false);
    declare_parameter("reedsSheppStepSize", 0.0); // <= 0 uses half the grid resolution
    declare_parameter("reedsSheppMaxShortcut", 0); // <= 0 = unlimited
    declare_parameter("useReedsSheppGoalShot", false); // requires useReedsSheppFinalPath
    declare_parameter("reedsSheppGoalShotMaxDistance", 15.0);

    declare_parameter("cellSkipFactor", 0.1);
    declare_parameter("destinationCircleRadius", 6);
    declare_parameter("generateBackwardMotions", true);
    declare_parameter("generateForwardMotions", true);
    declare_parameter("generateLateralMotions", false);
    declare_parameter("generatePointTurnMotions", true);
    declare_parameter("numAngles", 16);
    declare_parameter("numEndAngles", 8);
    declare_parameter("splineOrder", 4);

    declare_parameter("allowForwardDownhill", true);
    declare_parameter("articulatedSuspension", true); // if false, rigid chassis/axles model
    declare_parameter("enableInclineLimitting", false);
    declare_parameter("inclineLimittingLimit", 0.1);
    declare_parameter("inclineLimittingMinSlope", 0.2);
    declare_parameter("costFunctionDist", 0.0);
    declare_parameter("distToGround", 0.0);
    declare_parameter("initialPatchVariance", 0.0001);
    declare_parameter("maxSlope", 0.45);
    declare_parameter("maxStepHeight", 0.2);
    declare_parameter("minTraversablePercentage", 0.4);
    declare_parameter("robotHeight", 1.7);
    declare_parameter("robotSizeX", 0.80);
    declare_parameter("robotSizeY", 0.80);
    declare_parameter("slopeMetric", "NONE"); // Options: NONE, AVG_SLOPE, MAX_SLOPE, TRIANGLE_SLOPE
    declare_parameter("slopeMetricScale", 1.0);
    declare_parameter("obstacleInflationMultiplier", 1.0);
    declare_parameter("partiallyTraversableMultiplier", 2.0);
    declare_parameter("numYawSamples", 12); // yaw samples over [0,180) for partial-cell orientations
    declare_parameter("extend_trajectory", false);    
    declare_parameter("extension_distance", 0.0);    
}

void PathPlannerNode::updateParameters(){

    spline_primitive_config.gridSize                 = get_parameter("grid_resolution").as_double();
    spline_primitive_config.numAngles                = get_parameter("numAngles").as_int();
    spline_primitive_config.numEndAngles             = get_parameter("numEndAngles").as_int();
    spline_primitive_config.destinationCircleRadius  = get_parameter("destinationCircleRadius").as_int();
    spline_primitive_config.cellSkipFactor           = get_parameter("cellSkipFactor").as_double();
    spline_primitive_config.generateForwardMotions   = get_parameter("generateForwardMotions").as_bool();
    spline_primitive_config.generatePointTurnMotions = get_parameter("generatePointTurnMotions").as_bool();
    spline_primitive_config.generateLateralMotions   = get_parameter("generateLateralMotions").as_bool();
    spline_primitive_config.generateBackwardMotions  = get_parameter("generateBackwardMotions").as_bool();
    spline_primitive_config.splineOrder              = get_parameter("splineOrder").as_int();
    
    mobility_config.translationSpeed                      = get_parameter("translationSpeed").as_double();
    mobility_config.rotationSpeed                         = get_parameter("rotationSpeed").as_double();
    mobility_config.minTurningRadius                      = get_parameter("minTurningRadius").as_double();
    mobility_config.searchRadius                          = get_parameter("searchRadius").as_double();
    mobility_config.multiplierForward                     = get_parameter("multiplierForward").as_double();
    mobility_config.multiplierBackward                    = get_parameter("multiplierBackward").as_double();
    mobility_config.multiplierLateral                     = get_parameter("multiplierLateral").as_double();
    mobility_config.multiplierBackwardTurn                = get_parameter("multiplierBackwardTurn").as_double();
    mobility_config.multiplierForwardTurn                 = get_parameter("multiplierForwardTurn").as_double();
    mobility_config.multiplierPointTurn                   = get_parameter("multiplierPointTurn").as_double();
    mobility_config.multiplierLateralCurve                = get_parameter("multiplierLateralCurve").as_double();
    mobility_config.searchProgressSteps                   = get_parameter("searchProgressSteps").as_double();
    mobility_config.maxMotionCurveLength                  = get_parameter("maxMotionCurveLength").as_double();
    mobility_config.spline_sampling_resolution            = get_parameter("spline_sampling_resolution").as_double();
    mobility_config.remove_goal_offset                    = get_parameter("remove_goal_offset").as_bool();
    mobility_config.curvaturePenaltyWeight                = get_parameter("curvaturePenaltyWeight").as_double();
    mobility_config.angularCostWeight                     = get_parameter("angularCostWeight").as_double();

    traversability_config.gridResolution            = get_parameter("grid_resolution").as_double();
    traversability_config.maxSlope                  = get_parameter("maxSlope").as_double();
    traversability_config.maxStepHeight             = get_parameter("maxStepHeight").as_double();
    traversability_config.robotSizeX                = get_parameter("robotSizeX").as_double();
    traversability_config.robotSizeY                = get_parameter("robotSizeY").as_double();
    traversability_config.robotHeight               = get_parameter("robotHeight").as_double();
    traversability_config.slopeMetricScale          = get_parameter("slopeMetricScale").as_double();
    const std::string slope_metric = get_parameter("slopeMetric").as_string();
    if (slope_metric == "AVG_SLOPE")
        traversability_config.slopeMetric = traversability_generator3d::SlopeMetric::AVG_SLOPE;
    else if (slope_metric == "MAX_SLOPE")
        traversability_config.slopeMetric = traversability_generator3d::SlopeMetric::MAX_SLOPE;
    else if (slope_metric == "TRIANGLE_SLOPE")
        traversability_config.slopeMetric = traversability_generator3d::SlopeMetric::TRIANGLE_SLOPE;
    else
        traversability_config.slopeMetric = traversability_generator3d::SlopeMetric::NONE;
    traversability_config.inclineLimittingMinSlope  = get_parameter("inclineLimittingMinSlope").as_double(); 
    traversability_config.inclineLimittingLimit     = get_parameter("inclineLimittingLimit").as_double();
    traversability_config.costFunctionDist          = get_parameter("costFunctionDist").as_double();
    traversability_config.distToGround              = get_parameter("distToGround").as_double();
    traversability_config.minTraversablePercentage  = get_parameter("minTraversablePercentage").as_double();
    traversability_config.initialPatchVariance      = get_parameter("initialPatchVariance").as_double();
    traversability_config.allowForwardDownhill      = get_parameter("allowForwardDownhill").as_bool();
    traversability_config.articulatedSuspension     = get_parameter("articulatedSuspension").as_bool();
    traversability_config.enableInclineLimitting    = get_parameter("enableInclineLimitting").as_bool();
    traversability_config.obstacleInflationMultiplier = get_parameter("obstacleInflationMultiplier").as_double();
    traversability_config.partiallyTraversableMultiplier = get_parameter("partiallyTraversableMultiplier").as_double();
    traversability_config.numYawSamples              = get_parameter("numYawSamples").as_int();

    planner_config.epsilonSteps                     = get_parameter("epsilonSteps").as_int();
    planner_config.initialEpsilon                   = get_parameter("initialEpsilon").as_int();
    planner_config.numThreads                       = get_parameter("numThreads").as_int();
    planner_config.usePathStatistics                = get_parameter("usePathStatistics").as_bool();
    planner_config.searchUntilFirstSolution         = get_parameter("searchUntilFirstSolution").as_bool();
    planner_config.corridorWidth                    = get_parameter("corridorWidth").as_double();
    planner_config.goalOrientationMargin            = get_parameter("goalOrientationMargin").as_double();
    planner_config.goalDistanceMargin               = get_parameter("goalDistanceMargin").as_double();
    planner_config.maxTime                          = get_parameter("maxTime").as_double();
    planner_config.useReedsSheppFinalPath           = get_parameter("useReedsSheppFinalPath").as_bool();
    planner_config.reedsSheppStepSize               = get_parameter("reedsSheppStepSize").as_double();
    planner_config.reedsSheppMaxShortcut            = get_parameter("reedsSheppMaxShortcut").as_int();
    planner_config.useReedsSheppGoalShot            = get_parameter("useReedsSheppGoalShot").as_bool();
    planner_config.reedsSheppGoalShotMaxDistance    = get_parameter("reedsSheppGoalShotMaxDistance").as_double();
}

void PathPlannerNode::updateParametersFromConfigs(
    const sbpl_spline_primitives::SplinePrimitivesConfig& spline,
    const ugv_nav4d::Mobility& mobility,
    const traversability_generator3d::TraversabilityConfig& trav,
    const ugv_nav4d::PlannerConfig& planner)
{
    // Map slopeMetric enum -> string for the parameter server.
    std::string slope_metric;
    switch (trav.slopeMetric)
    {
        case traversability_generator3d::SlopeMetric::AVG_SLOPE:      slope_metric = "AVG_SLOPE"; break;
        case traversability_generator3d::SlopeMetric::MAX_SLOPE:      slope_metric = "MAX_SLOPE"; break;
        case traversability_generator3d::SlopeMetric::TRIANGLE_SLOPE: slope_metric = "TRIANGLE_SLOPE"; break;
        default:                                                      slope_metric = "NONE"; break;
    }

    std::vector<rclcpp::Parameter> params = {
        // Shared grid resolution (spline.gridSize == trav.gridResolution)
        rclcpp::Parameter("grid_resolution", trav.gridResolution),
        // Spline primitives
        rclcpp::Parameter("numAngles", spline.numAngles),
        rclcpp::Parameter("numEndAngles", spline.numEndAngles),
        rclcpp::Parameter("destinationCircleRadius", spline.destinationCircleRadius),
        rclcpp::Parameter("cellSkipFactor", spline.cellSkipFactor),
        rclcpp::Parameter("generateForwardMotions", spline.generateForwardMotions),
        rclcpp::Parameter("generatePointTurnMotions", spline.generatePointTurnMotions),
        rclcpp::Parameter("generateLateralMotions", spline.generateLateralMotions),
        rclcpp::Parameter("generateBackwardMotions", spline.generateBackwardMotions),
        rclcpp::Parameter("splineOrder", spline.splineOrder),
        // Mobility (multipliers are declared as doubles on the parameter server)
        rclcpp::Parameter("translationSpeed", mobility.translationSpeed),
        rclcpp::Parameter("rotationSpeed", mobility.rotationSpeed),
        rclcpp::Parameter("minTurningRadius", mobility.minTurningRadius),
        rclcpp::Parameter("searchRadius", mobility.searchRadius),
        rclcpp::Parameter("searchProgressSteps", mobility.searchProgressSteps),
        rclcpp::Parameter("multiplierForward", (double)mobility.multiplierForward),
        rclcpp::Parameter("multiplierBackward", (double)mobility.multiplierBackward),
        rclcpp::Parameter("multiplierLateral", (double)mobility.multiplierLateral),
        rclcpp::Parameter("multiplierForwardTurn", (double)mobility.multiplierForwardTurn),
        rclcpp::Parameter("multiplierBackwardTurn", (double)mobility.multiplierBackwardTurn),
        rclcpp::Parameter("multiplierPointTurn", (double)mobility.multiplierPointTurn),
        rclcpp::Parameter("multiplierLateralCurve", (double)mobility.multiplierLateralCurve),
        rclcpp::Parameter("maxMotionCurveLength", mobility.maxMotionCurveLength),
        rclcpp::Parameter("spline_sampling_resolution", mobility.spline_sampling_resolution),
        rclcpp::Parameter("remove_goal_offset", mobility.remove_goal_offset),
        rclcpp::Parameter("curvaturePenaltyWeight", mobility.curvaturePenaltyWeight),
        rclcpp::Parameter("angularCostWeight", mobility.angularCostWeight),
        // Traversability
        rclcpp::Parameter("maxSlope", trav.maxSlope),
        rclcpp::Parameter("maxStepHeight", trav.maxStepHeight),
        rclcpp::Parameter("robotSizeX", trav.robotSizeX),
        rclcpp::Parameter("robotSizeY", trav.robotSizeY),
        rclcpp::Parameter("robotHeight", trav.robotHeight),
        rclcpp::Parameter("slopeMetricScale", trav.slopeMetricScale),
        rclcpp::Parameter("slopeMetric", slope_metric),
        rclcpp::Parameter("inclineLimittingMinSlope", trav.inclineLimittingMinSlope),
        rclcpp::Parameter("inclineLimittingLimit", trav.inclineLimittingLimit),
        rclcpp::Parameter("costFunctionDist", trav.costFunctionDist),
        rclcpp::Parameter("distToGround", trav.distToGround),
        rclcpp::Parameter("minTraversablePercentage", trav.minTraversablePercentage),
        rclcpp::Parameter("initialPatchVariance", trav.initialPatchVariance),
        rclcpp::Parameter("allowForwardDownhill", trav.allowForwardDownhill),
        rclcpp::Parameter("articulatedSuspension", trav.articulatedSuspension),
        rclcpp::Parameter("enableInclineLimitting", trav.enableInclineLimitting),
        rclcpp::Parameter("obstacleInflationMultiplier", trav.obstacleInflationMultiplier),
        rclcpp::Parameter("partiallyTraversableMultiplier", trav.partiallyTraversableMultiplier),
        rclcpp::Parameter("numYawSamples", trav.numYawSamples),
        // Planner (epsilonSteps/initialEpsilon/numThreads are declared as ints)
        rclcpp::Parameter("epsilonSteps", (int)planner.epsilonSteps),
        rclcpp::Parameter("initialEpsilon", (int)planner.initialEpsilon),
        rclcpp::Parameter("numThreads", (int)planner.numThreads),
        rclcpp::Parameter("usePathStatistics", planner.usePathStatistics),
        rclcpp::Parameter("searchUntilFirstSolution", planner.searchUntilFirstSolution),
        rclcpp::Parameter("corridorWidth", planner.corridorWidth),
        rclcpp::Parameter("maxTime", planner.maxTime),
        rclcpp::Parameter("goalOrientationMargin", planner.goalOrientationMargin),
        rclcpp::Parameter("goalDistanceMargin", planner.goalDistanceMargin),
        rclcpp::Parameter("useReedsSheppFinalPath", planner.useReedsSheppFinalPath),
        rclcpp::Parameter("reedsSheppStepSize", planner.reedsSheppStepSize),
        rclcpp::Parameter("reedsSheppMaxShortcut", planner.reedsSheppMaxShortcut),
        rclcpp::Parameter("useReedsSheppGoalShot", planner.useReedsSheppGoalShot),
        rclcpp::Parameter("reedsSheppGoalShotMaxDistance", planner.reedsSheppGoalShotMaxDistance),
    };

    // Setting the parameters triggers parametersCallback(), which queues them so the
    // parameterUpdateTimerCallback() rebuilds the planner via configurePlanner().
    set_parameters(params);
    RCLCPP_INFO(this->get_logger(), "Applied %zu parameters from GUI; planner will reconfigure.", params.size());
}

void PathPlannerNode::configurePlanner(){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    // If the grid resolution changed, the MLS map must be recreated at the new resolution
    // before the traversability map and planner are rebuilt below.
    const double requested_res = get_parameter("grid_resolution").as_double();
    if (std::abs(requested_res - current_grid_resolution) > 1e-9){
        RCLCPP_INFO_STREAM(this->get_logger(), "grid_resolution changed ("
                           << current_grid_resolution << " -> " << requested_res
                           << "); recreating MLS map.");
        initializeMLSMap();
    }

    updateParameters();
    planner_ptr.reset(new ugv_nav4d::Planner(spline_primitive_config, traversability_config, mobility_config, planner_config));
    traversability_generator_ptr.reset(new traversability_generator3d::TraversabilityGenerator3d(traversability_config));
    is_configured = true;

    if(got_map){//If map is already available then load the last known map.
        if (!get_parameter("read_pose_from_topic").as_bool())
        {
            if (!updatePoseFromTF()){
                RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot determine start pose: TF lookup "
                                << get_parameter("world_frame").as_string() << " <- "
                                << get_parameter("robot_frame").as_string() << " failed.");
                return;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Planner state: Configuring with the already-loaded map (no disk access)");
        publishStatus("Generating traversability map...");
        traversability_generator_ptr->setMLSGrid(mls_map_ptr);

        Eigen::Affine3d body2MLS;
        body2MLS.translation() << start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z;
        Eigen::Quaterniond quat(start_pose.pose.orientation.w,
                                start_pose.pose.orientation.x,
                                start_pose.pose.orientation.y,
                                start_pose.pose.orientation.z);
        body2MLS.linear() = quat.toRotationMatrix();
        Eigen::Affine3d body2Ground(Eigen::Affine3d::Identity());
        body2Ground.translation() = Eigen::Vector3d(0, 0, -get_parameter("distToGround").as_double());
        Eigen::Affine3d ground2Mls(body2MLS * body2Ground);

        auto startPosition = ground2Mls.translation();
        traversability_generator_ptr->expandAll(startPosition);
        applyForbiddenZones();
        auto travMap = traversability_generator_ptr->getTraversabilityMap();
        planner_ptr->updateMap(travMap);
        RCLCPP_INFO(this->get_logger(), "Planner state: Ready");
        publishStatus("Ready");
        if (map_update_callback) {
            map_update_callback();
        }
    }
}

bool PathPlannerNode::publishMaps(){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (!is_configured){
        configurePlanner();
    }

    return publishMLSMap() && publishTravMap();
}

bool PathPlannerNode::publishMLSMap(){
    ugv_nav4d_ros2::msg::MLSMap map_msg;    
    map_msg.width = 1;
    map_msg.height = 1;
    map_msg.depth = 1;
    map_msg.resolution = get_parameter("grid_resolution").as_double();
    map_msg.header.frame_id = get_parameter("world_frame").as_string();

    maps::grid::Vector2ui num_cell = mls_map_ptr->getNumCells();
    typedef maps::grid::MLSMap<maps::grid::MLSConfig::SLOPE>::CellType Cell;

    for (size_t x = 0; x < num_cell.x(); x++)
    {
        for (size_t y = 0; y < num_cell.y(); y++)
        {
            const Cell &list = mls_map_ptr->at(x, y);
            for (Cell::const_iterator it = list.begin(); it != list.end(); it++)
            {
                const maps::grid::SurfacePatch<maps::grid::MLSConfig::SLOPE>& p = *it;  
                float minZ, maxZ;
                p.getRange(minZ, maxZ);
                minZ -= 5e-4f;
                maxZ += 5e-4f;
                Eigen::Vector3f normal = p.getNormal();
                if(normal.z() < 0)
                    normal *= -1.0;

                ugv_nav4d_ros2::msg::MLSPatch patch_msg;
                maps::grid::Vector2d pos(0.00, 0.00);

                // Calculate the position of the cell center.
                pos = (maps::grid::Index(x, y).cast<double>() + maps::grid::Vector2d(0.5, 0.5)).array() * mls_map_ptr->getResolution().array();
                patch_msg.position.x = pos.x() + mls_min_x;
                patch_msg.position.y = pos.y() + mls_min_y;
                patch_msg.position.z = p.getCenter().z();

                if(normal.allFinite())
                {
                    patch_msg.type = "plane";
                    Eigen::Hyperplane<float, 3> plane = Eigen::Hyperplane<float, 3>(normal, p.getCenter());
    
                    patch_msg.a = plane.normal()(0);
                    patch_msg.b = plane.normal()(1);
                    patch_msg.c = plane.normal()(2);
                    patch_msg.d = -plane.offset();          
                    patch_msg.height = (maxZ - minZ) + 1e-3f;
                    patch_msg.minz = minZ;          
                    patch_msg.maxz = maxZ;          
             
                    map_msg.patches.push_back(patch_msg);
                }
                else
                {
                    patch_msg.type = "box";
                    patch_msg.depth = 0.5;
                    patch_msg.height = (maxZ - minZ) + 1e-3f;
                    patch_msg.a = 0;
                    patch_msg.b = 0;
                    patch_msg.c = 1;
                    patch_msg.d = 0; 
                    map_msg.patches.push_back(patch_msg);
                }
            }
        }
    }

    if (map_msg.patches.size() == 0){
        RCLCPP_WARN(this->get_logger(), "Cannot publish MLS map: map is empty.");
        return false;
    }

    mls_map_publisher->publish(map_msg);
    return true;
}

bool PathPlannerNode::publishTravMap(){
    const auto& trav_map_3d = traversability_generator_ptr->getTraversabilityMap(); 
    ugv_nav4d_ros2::msg::TravMap map_msg;
    map_msg.width = 1;
    map_msg.height = 1;
    map_msg.depth = 1;
    map_msg.resolution = get_parameter("grid_resolution").as_double();
    map_msg.header.frame_id = get_parameter("world_frame").as_string();

    double dist_min_x = get_parameter("dist_min_x").as_int();
    double dist_min_y = get_parameter("dist_min_y").as_int();

    for(const maps::grid::LevelList<traversability_generator3d::TravGenNode *> &l : trav_map_3d)
    {
        for(const traversability_generator3d::TravGenNode *n : l)
        {
            ugv_nav4d_ros2::msg::TravPatch patch_msg;

            Eigen::Vector3d position;
            trav_map_3d.fromGrid(n->getIndex(), position, n->getHeight(), false);
            patch_msg.a = n->getUserData().plane.normal()(0);
            patch_msg.b = n->getUserData().plane.normal()(1);
            patch_msg.c = n->getUserData().plane.normal()(2);
            patch_msg.d = -n->getUserData().plane.offset();
            patch_msg.position.x = position.x();
            patch_msg.position.y = position.y();
            patch_msg.position.z = position.z();

            switch(n->getUserData().nodeType){
                case traversability_generator3d::NodeType::TRAVERSABLE:
                    patch_msg.color.r = 0;
                    patch_msg.color.g = 1;
                    patch_msg.color.b = 0;
                    patch_msg.color.a = 1;
                    break;                    
                case traversability_generator3d::NodeType::OBSTACLE:
                    patch_msg.color.r = 1;
                    patch_msg.color.g = 0;
                    patch_msg.color.b = 0;
                    patch_msg.color.a = 1;
                    break;    
                case traversability_generator3d::NodeType::FRONTIER:
                    patch_msg.color.r = 0;
                    patch_msg.color.g = 0;
                    patch_msg.color.b = 1;
                    patch_msg.color.a = 1;
                    break;   
                case traversability_generator3d::NodeType::UNSET:
                    patch_msg.color.r = 1;
                    patch_msg.color.g = 1;
                    patch_msg.color.b = 0;
                    patch_msg.color.a = 1;
                    break;   
                case traversability_generator3d::NodeType::UNKNOWN:
                    patch_msg.color.r = 0.5;
                    patch_msg.color.g = 0;
                    patch_msg.color.b = 0.5;
                    patch_msg.color.a = 1;
                    break;

                case traversability_generator3d::NodeType::INFLATED_OBSTACLE:
                    patch_msg.color.r = 1.0;
                    patch_msg.color.g = 0.5;
                    patch_msg.color.b = 0.0;
                    patch_msg.color.a = 1;
                    break;

                case traversability_generator3d::NodeType::INFLATED_FRONTIER:
                    patch_msg.color.r = 0.5;
                    patch_msg.color.g = 0.8;
                    patch_msg.color.b = 1.0;
                    patch_msg.color.a = 1;
                    break;

                case traversability_generator3d::NodeType::PARTIALLY_TRAVERSABLE:
                    // yellow-green, same as TravMap3dVisualization
                    patch_msg.color.r = 0.6;
                    patch_msg.color.g = 0.8;
                    patch_msg.color.b = 0.0;
                    patch_msg.color.a = 1;
                    break;

                default: // e.g. HOLE
                    patch_msg.color.r = 0.3;
                    patch_msg.color.g = 0.3;
                    patch_msg.color.b = 0.3;
                    patch_msg.color.a = 1;
                    break;
            }
            map_msg.patches.push_back(patch_msg);

        }
    }


    if (map_msg.patches.size() == 0){
        RCLCPP_WARN(this->get_logger(), "Cannot publish traversability map: map is empty (it is generated on configure/plan).");
        return false;
    }

    trav_map_publisher->publish(map_msg);
    return true;
}

void PathPlannerNode::triggerPlanningFromGUI(const base::Pose& /*start*/, const base::Pose& goal)
{
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (!is_configured){
        configurePlanner();
    }

    if (!got_map){
        RCLCPP_WARN_STREAM(this->get_logger(), "GUI planning request rejected: no map available yet.");
        publishStatus("Planning request rejected: no map available yet");
        return;
    }

    if (is_planning){
        RCLCPP_WARN_STREAM(this->get_logger(), "GUI planning request rejected: planner is busy with a previous request.");
        publishStatus("Planning request rejected: planner is busy");
        return;
    }

    // The start pose always comes from the ROS node (robot's actual pose), not the GUI:
    // from TF, or from the /start_pose topic when read_pose_from_topic is set. Only the
    // goal is taken from the GUI.
    if (!get_parameter("read_pose_from_topic").as_bool())
    {
        if (!updatePoseFromTF()){
            RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot determine start pose: TF lookup "
                                << get_parameter("world_frame").as_string() << " <- "
                                << get_parameter("robot_frame").as_string() << " failed.");
            return;
        }
    }

    start_pose_rbs.position = Eigen::Vector3d(start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z);
    start_pose_rbs.orientation = Eigen::Quaterniond(start_pose.pose.orientation.w, start_pose.pose.orientation.x, start_pose.pose.orientation.y, start_pose.pose.orientation.z);

    goal_pose_rbs.position = goal.position;
    goal_pose_rbs.orientation = goal.orientation;

    plan();
}

} // namespace ugv_nav4d_ros2
