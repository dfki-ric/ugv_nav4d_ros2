#include "ugv_nav4d_ros2.hpp"
#include <unordered_map>
#include <unordered_set>
#include "util_functions.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h> // For removeNaNFromPointCloud
#include <pcl/filters/voxel_grid.h> // For optional PLY downsampling

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <algorithm>
#include <array>
#include <fstream>
#include <cmath>
#include <chrono>
#include <cctype>
#include <deque>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unordered_set>

#ifdef _OPENMP
#include <omp.h>
#endif

using namespace rclcpp;

namespace ugv_nav4d_ros2 {

namespace {

bool zoneAffectsPlanning(const ugv_nav4d_ros2::msg::ForbiddenZone& zone)
{
    // TRAVERSABLE zones are pure MLS-edit regions: drawing one changes nothing
    // until the operator runs Delete/Fill, and planning only changes on the
    // explicit travmap regeneration. So they never force a rebuild here.
    return zone.zone_type != ugv_nav4d_ros2::msg::ForbiddenZone::SPEED_LIMIT &&
           zone.zone_type != ugv_nav4d_ros2::msg::ForbiddenZone::ANNOTATION &&
           zone.zone_type != ugv_nav4d_ros2::msg::ForbiddenZone::TRAVERSABLE;
}

std::string normalizedFrame(std::string frame)
{
    while (!frame.empty() && frame.front() == '/'){
        frame.erase(frame.begin());
    }
    return frame;
}

} // namespace

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
    // Previews get their own topics so a fresh plan never visually replaces the
    // route the follower is currently driving; Execute promotes the preview onto
    // the executing topics above.
    preview_path_publisher = this->create_publisher<nav_msgs::msg::Path>("/ugv_nav4d_ros2/preview_path", 10);
    preview_colored_path_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/ugv_nav4d_ros2/preview_mission_path", rclcpp::QoS(1).transient_local());
    // Latched: the map is published only on discrete events (load, regenerate,
    // zone edits), so late-joining subscribers (field_operations readiness,
    // RViz) must still receive the last map without a manual republish.
    trav_map_publisher = this->create_publisher<ugv_nav4d_ros2::msg::TravMap>(
            "/ugv_nav4d_ros2/trav_map", rclcpp::QoS(1).transient_local());
    mls_map_publisher = this->create_publisher<ugv_nav4d_ros2::msg::MLSMap>(
            "/ugv_nav4d_ros2/mls_map", rclcpp::QoS(1).transient_local());
    robot_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/ugv_nav4d_ros2/robot_pose", 10);
    zone_speed_limit_publisher = this->create_publisher<std_msgs::msg::Float32>(
            "/ugv_nav4d_ros2/zone_speed_limit", 10);
    footprint_info_publisher = this->create_publisher<std_msgs::msg::String>(
        "/ugv_nav4d_ros2/footprint_info",
        rclcpp::QoS(1).transient_local());
    height_info_publisher = this->create_publisher<std_msgs::msg::String>(
        "/ugv_nav4d_ros2/height_info", 10);
    rebuilding_publisher = this->create_publisher<std_msgs::msg::Bool>(
        "/ugv_nav4d_ros2/rebuilding", rclcpp::QoS(1).transient_local());
    {
        std_msgs::msg::Bool not_rebuilding;
        not_rebuilding.data = false;
        rebuilding_publisher->publish(not_rebuilding);
    }
    mls_delete_last_zone_service = this->create_service<ugv_nav4d_ros2::srv::DeleteMlsPatches>(
        "/ugv_nav4d_ros2/mls_delete_last_zone",
        [this](const std::shared_ptr<ugv_nav4d_ros2::srv::DeleteMlsPatches::Request> request,
               std::shared_ptr<ugv_nav4d_ros2::srv::DeleteMlsPatches::Response> response){
            std::lock_guard<std::recursive_mutex> lock(planner_mutex);
            response->success = false;
            if (is_planning){
                response->message = "Cannot edit the MLS while planning is active.";
            } else if (!got_map || !mls_map_ptr){
                response->message = "No map loaded; nothing to edit.";
            } else if (!std::isfinite(request->top_m) || request->top_m < 0.0 || request->top_m > 10.0){
                response->message = "Deletion ceiling rejected: must be 0 (default) to 10 m.";
            } else if (auto* zone = lastTraversableZone()){
                zone->fill_enabled = false;
                zone->delete_top = static_cast<float>(request->top_m);
                last_mls_patches_removed_ = 0;
                last_mls_patches_added_ = 0;
                applyMlsEditZone(*zone);
                publishMaps();
                response->success = true;
                response->message = "Deleted " + std::to_string(last_mls_patches_removed_) +
                                    " MLS patch(es) (MLS only); click 'Regenerate trav map' to apply to planning.";
            } else {
                response->message = "No Traversable (MLS edit) zone drawn; draw one with the zone tool first.";
            }
            publishStatus(response->message);
        });
    mls_fill_last_zone_service = this->create_service<ugv_nav4d_ros2::srv::SetFillPlane>(
        "/ugv_nav4d_ros2/mls_fill_last_zone",
        [this](const std::shared_ptr<ugv_nav4d_ros2::srv::SetFillPlane::Request> request,
               std::shared_ptr<ugv_nav4d_ros2::srv::SetFillPlane::Response> response){
            std::lock_guard<std::recursive_mutex> lock(planner_mutex);
            response->success = false;
            if (is_planning){
                response->message = "Cannot edit the MLS while planning is active.";
            } else if (!got_map || !mls_map_ptr){
                response->message = "No map loaded; nothing to fill.";
            } else if (!std::isfinite(request->z_offset) || !std::isfinite(request->roll_deg) ||
                       !std::isfinite(request->pitch_deg) || std::abs(request->roll_deg) > 45.0 ||
                       std::abs(request->pitch_deg) > 45.0){
                response->message = "Fill plane rejected: offsets must be finite, tilt within +/-45 degrees.";
            } else if (auto* zone = lastTraversableZone()){
                // With auto_fit the request values are trim offsets on top of a
                // plane fitted to the ground around the polygon; without it they
                // are the plane itself (original behavior).
                double fit_roll = 0.0, fit_pitch = 0.0, fit_z = 0.0;
                std::string fit_err;
                if (request->auto_fit &&
                    !fitFillPlaneToSurroundingGround(*zone, fit_roll, fit_pitch, fit_z, fit_err)){
                    response->message = "Fit to ground failed: " + fit_err;
                    publishStatus(response->message);
                    return;
                }
                const double roll = fit_roll + request->roll_deg * M_PI / 180.0;
                const double pitch = fit_pitch + request->pitch_deg * M_PI / 180.0;
                const double z_offset = fit_z + request->z_offset;
                if (std::abs(roll) > 45.0 * M_PI / 180.0 || std::abs(pitch) > 45.0 * M_PI / 180.0){
                    response->message = "Fill plane rejected: fitted tilt plus trim exceeds +/-45 degrees.";
                    publishStatus(response->message);
                    return;
                }
                zone->fill_enabled = true;
                zone->fill_z_offset = static_cast<float>(z_offset);
                zone->fill_roll = static_cast<float>(roll);
                zone->fill_pitch = static_cast<float>(pitch);
                zone->delete_top = (std::isfinite(request->delete_top_m) &&
                                    request->delete_top_m > 0.0 && request->delete_top_m <= 10.0)
                                   ? static_cast<float>(request->delete_top_m) : 0.0f;
                last_mls_patches_removed_ = 0;
                last_mls_patches_added_ = 0;
                applyMlsEditZone(*zone);
                publishMaps();
                response->success = true;
                response->applied_z_offset = z_offset;
                response->applied_roll_deg = roll * 180.0 / M_PI;
                response->applied_pitch_deg = pitch * 180.0 / M_PI;
                std::ostringstream msg;
                msg << "Fill applied";
                if (request->auto_fit){
                    msg << std::fixed << std::setprecision(1)
                        << " (fit: z " << std::setprecision(2) << z_offset << " m"
                        << std::setprecision(1)
                        << ", roll " << response->applied_roll_deg
                        << ", pitch " << response->applied_pitch_deg << " deg)";
                }
                msg << ": removed " << last_mls_patches_removed_
                    << ", added " << last_mls_patches_added_
                    << " patch(es) (MLS only); click 'Regenerate trav map' to apply to planning.";
                response->message = msg.str();
            } else {
                response->message = "No Traversable (MLS edit) zone drawn; draw one with the zone tool first.";
            }
            publishStatus(response->message);
        });
    regenerate_travmap_service = this->create_service<std_srvs::srv::Trigger>(
            "/ugv_nav4d_ros2/regenerate_travmap",
            std::bind(&PathPlannerNode::regenerateTravMapCallback, this, std::placeholders::_1, std::placeholders::_2));
    groom_state_publisher = create_publisher<std_msgs::msg::Bool>(
        "/ugv_nav4d_ros2/groom_under_robot", rclcpp::QoS(1).transient_local());
    {
        std_msgs::msg::Bool initial_groom;
        initial_groom.data = false;
        groom_state_publisher->publish(initial_groom);
    }
    set_groom_service = this->create_service<std_srvs::srv::SetBool>(
        "/ugv_nav4d_ros2/set_groom_under_robot",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response){
            std::lock_guard<std::recursive_mutex> lock(planner_mutex);
            groom_under_robot_ = request->data;
            // Force an immediate groom at the current spot on enable.
            last_groom_x_ = std::numeric_limits<double>::quiet_NaN();
            std_msgs::msg::Bool state;
            state.data = groom_under_robot_;
            groom_state_publisher->publish(state);
            response->success = true;
            response->message = groom_under_robot_
                ? "Grooming ON: the MLS is flattened under the wheels as the robot moves "
                  "(pure MLS edit; regenerate the trav map to apply, save the map to keep)."
                : "Grooming OFF (totals this session: removed "
                  + std::to_string(groom_removed_total_) + ", added "
                  + std::to_string(groom_added_total_) + " patches).";
            publishStatus(response->message);
        });
    calibrate_geometry_service = this->create_service<std_srvs::srv::Trigger>(
        "/ugv_nav4d_ros2/calibrate_geometry",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response){
            // Composite after-adaptation calibration: footprint measurement
            // and height recalibration back-to-back. Both mark the planner
            // dirty via set_parameters; the parameter timer batches them into
            // ONE configurePlanner (map regeneration + expansion). It cannot
            // fire in between: single-threaded executor.
            std::lock_guard<std::recursive_mutex> lock(planner_mutex);
            auto footprint_response = std::make_shared<std_srvs::srv::Trigger::Response>();
            updateFootprintCallback(request, footprint_response);
            auto height_response = std::make_shared<std_srvs::srv::Trigger::Response>();
            recalibrateHeightCallback(request, height_response);
            response->success = footprint_response->success && height_response->success;
            response->message = "Footprint: " + footprint_response->message +
                                " | Height: " + height_response->message;
            publishStatus(response->message);
        });
    recalibrate_height_service = this->create_service<std_srvs::srv::Trigger>(
        "/ugv_nav4d_ros2/recalibrate_height",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response){
            recalibrateHeightCallback(request, response);
        });
    wheelbase_publisher = this->create_publisher<std_msgs::msg::Float32>(
            "/ugv_nav4d_ros2/wheelbase", 10);
    // footprint_planner = the footprint the PLANNER is configured with
    // (robotSizeX/Y @ footprintOffsetX, plus axle line and label markers);
    // footprint_real = the TRUE envelope measured live from TF (wheels + tool
    // + margins), i.e. what the update_footprint service would apply. The
    // operator compares the two in RViz to spot stale planner config.
    footprint_marker_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/ugv_nav4d_ros2/footprint_planner", 10);
    footprint_polygon_publisher = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "/ugv_nav4d_ros2/footprint_real", 10);
    // groom_footprint = the wheels-only + margins rectangle the grooming
    // feature flattens; empty polygon while grooming is off.
    groom_footprint_publisher = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "/ugv_nav4d_ros2/groom_footprint", 10);
    const double footprint_publish_period = get_parameter("footprint_publish_period").as_double();
    if (footprint_publish_period > 0.0){
        wheelbase_status_timer = this->create_wall_timer(
                std::chrono::duration<double>(footprint_publish_period),
                std::bind(&PathPlannerNode::publishWheelbaseStatus, this));
    }
    last_zone_speed_limit_match = this->get_clock()->now();

    setupSubscriptions();

    // Publish the TRUE (empty) startup state of every latched display topic.
    // The zenoh bridge caches transient_local samples and outlives planner and
    // rviz restarts; without these, a rejoining rviz receives the PREVIOUS
    // session's zones/waypoints/paths from the bridge cache as ghosts.
    publishWaypointMarkers();
    publishForbiddenZoneMarkers();
    clearExecutingPathDisplay();

    publishMissionStatus(ugv_nav4d_ros2::msg::MissionStatus::READY, "Planner node started");
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

    waypoint_photos_subscription = create_subscription<std_msgs::msg::String>(
        "/follow_path_client/waypoint_photos", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg){
            std::lock_guard<std::recursive_mutex> lock(planner_mutex);
            waypoint_photos_json = msg->data;
        });
    execution_status_subscription = create_subscription<ugv_nav4d_ros2::msg::MissionStatus>(
        "/ugv_nav4d_ros2/execution_status", rclcpp::QoS(1).transient_local(),
        [this](const ugv_nav4d_ros2::msg::MissionStatus::SharedPtr msg){
            // Tracked so the clear-executed-path service can refuse while a
            // route is actually being driven. Clearing itself is manual
            // (operator button) by explicit request.
            last_execution_state_ = msg->state;
        });
    sub_goal_pose = create_subscription<geometry_msgs::msg::PoseStamped>("/ugv_nav4d_ros2/goal_pose", 1,
            bind(&PathPlannerNode::processGoalRequest, this, std::placeholders::_1));

    // Waypoint queue: poses accumulated here are planned through (in order) when the
    // next goal arrives. The queue is only consumed by planning, never by time.
    sub_add_waypoint = create_subscription<geometry_msgs::msg::PoseStamped>("/ugv_nav4d_ros2/add_waypoint", 10,
            bind(&PathPlannerNode::addWaypointCallback, this, std::placeholders::_1));

    auto_plan_state_publisher = create_publisher<std_msgs::msg::Bool>(
        "/ugv_nav4d_ros2/auto_plan", rclcpp::QoS(1).transient_local());
    {
        std_msgs::msg::Bool initial_auto_plan;
        initial_auto_plan.data = false;
        auto_plan_state_publisher->publish(initial_auto_plan);
    }
    set_auto_plan_service = this->create_service<std_srvs::srv::SetBool>(
        "/ugv_nav4d_ros2/set_auto_plan",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response){
            std::lock_guard<std::recursive_mutex> lock(planner_mutex);
            auto_plan_enabled_ = request->data;
            std_msgs::msg::Bool state;
            state.data = auto_plan_enabled_;
            auto_plan_state_publisher->publish(state);
            response->success = true;
            if (auto_plan_enabled_){
                response->message = waypoint_queue.empty()
                    ? "Auto plan ON: the next waypoint plans a route to itself."
                    : "Auto plan ON: planning the current waypoint chain...";
                publishStatus(response->message);
                autoPlanIfEnabled();
            } else {
                response->message = "Auto plan OFF: waypoints wait for an explicit goal again.";
                publishStatus(response->message);
            }
        });
    clear_waypoints_service = this->create_service<std_srvs::srv::Trigger>(
            "/ugv_nav4d_ros2/clear_waypoints", std::bind(&PathPlannerNode::clearWaypointsCallback, this, std::placeholders::_1, std::placeholders::_2));

    remove_last_waypoint_service = this->create_service<std_srvs::srv::Trigger>(
            "/ugv_nav4d_ros2/remove_last_waypoint", std::bind(&PathPlannerNode::removeLastWaypointCallback, this, std::placeholders::_1, std::placeholders::_2));

    reverse_waypoints_service = this->create_service<std_srvs::srv::Trigger>(
            "/ugv_nav4d_ros2/reverse_waypoints", std::bind(&PathPlannerNode::reverseWaypointsCallback, this, std::placeholders::_1, std::placeholders::_2));

    edit_waypoint_service = this->create_service<ugv_nav4d_ros2::srv::EditWaypoint>(
            "/ugv_nav4d_ros2/edit_waypoint", std::bind(&PathPlannerNode::editWaypointCallback, this, std::placeholders::_1, std::placeholders::_2));

    // transient_local so RViz still shows the queued waypoints after a display restart
    waypoint_poses_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/ugv_nav4d_ros2/waypoint_poses", rclcpp::QoS(1).transient_local());
    waypoint_marker_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/ugv_nav4d_ros2/waypoint_markers", rclcpp::QoS(1).transient_local());

    // Operator feedback: latest planner status as a plain string (transient_local so a
    // late-joining RViz panel immediately sees the current state).
    status_publisher = this->create_publisher<std_msgs::msg::String>(
            "/ugv_nav4d_ros2/status", rclcpp::QoS(1).transient_local());
    mission_status_publisher = this->create_publisher<ugv_nav4d_ros2::msg::MissionStatus>(
            "/ugv_nav4d_ros2/planner_status", rclcpp::QoS(1).transient_local());
    route_risk_publisher = this->create_publisher<ugv_nav4d_ros2::msg::RouteRisk>(
            "/ugv_nav4d_ros2/route_risk", rclcpp::QoS(1).transient_local());
    route_valid_publisher = this->create_publisher<std_msgs::msg::Bool>(
            "/ugv_nav4d_ros2/route_valid", rclcpp::QoS(1).transient_local());
    // Latched "a fresh, not-yet-executed preview exists": the panel gates and
    // dispatches its Execute button on this (no preview -> Execute acts as
    // Resume) instead of on route_valid, which stays true for the whole
    // retained mission and made Execute look actionable after a bare pause.
    preview_pending_publisher = this->create_publisher<std_msgs::msg::Bool>(
            "/ugv_nav4d_ros2/preview_pending", rclcpp::QoS(1).transient_local());
    publishPreviewPending();

    save_map_service = this->create_service<ugv_nav4d_ros2::srv::MissionFile>(
            "/ugv_nav4d_ros2/save_mls_map", std::bind(&PathPlannerNode::saveMapCallback, this, std::placeholders::_1, std::placeholders::_2));
    inspect_traversability_service = this->create_service<ugv_nav4d_ros2::srv::InspectTraversability>(
            "/ugv_nav4d_ros2/inspect_traversability", std::bind(&PathPlannerNode::inspectTraversabilityCallback, this, std::placeholders::_1, std::placeholders::_2));
    replan_current_mission_service = this->create_service<std_srvs::srv::Trigger>(
            "/ugv_nav4d_ros2/replan_current_mission", std::bind(&PathPlannerNode::replanCurrentMissionCallback, this, std::placeholders::_1, std::placeholders::_2));
    recover_out_of_obstacle_service = this->create_service<std_srvs::srv::Trigger>(
            "/ugv_nav4d_ros2/recover_out_of_obstacle", std::bind(&PathPlannerNode::recoverOutOfObstacleCallback, this, std::placeholders::_1, std::placeholders::_2));
    save_mission_service = this->create_service<ugv_nav4d_ros2::srv::MissionFile>(
            "/ugv_nav4d_ros2/save_mission", std::bind(&PathPlannerNode::saveMissionCallback, this, std::placeholders::_1, std::placeholders::_2));
    load_mission_service = this->create_service<ugv_nav4d_ros2::srv::MissionFile>(
            "/ugv_nav4d_ros2/load_mission", std::bind(&PathPlannerNode::loadMissionCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Orientation inspection: the rviz tool publishes a polygon, the node
    // answers with allowed-orientation wedges for the partially traversable
    // cells inside it. Latched so a display restart keeps the last answer.
    inspect_orientations_sub = create_subscription<geometry_msgs::msg::PolygonStamped>(
            "/ugv_nav4d_ros2/inspect_orientations_region", 10,
            std::bind(&PathPlannerNode::inspectOrientationsCallback, this, std::placeholders::_1));
    allowed_orientation_marker_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/ugv_nav4d_ros2/allowed_orientation_markers", rclcpp::QoS(1).transient_local());

    // Execution gate: planning only publishes a preview (path / labeled_path_segments /
    // colored_path). The follower listens on execute_path_segments, which is only
    // published when the operator confirms via the execute_path service.
    execute_path_publisher = this->create_publisher<ugv_nav4d_ros2::msg::LabeledPathArray>(
            "/ugv_nav4d_ros2/execute_path_segments", 10);

    clear_executed_path_service = this->create_service<std_srvs::srv::Trigger>(
        "/ugv_nav4d_ros2/clear_executed_path",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response){
            if (last_execution_state_ == ugv_nav4d_ros2::msg::MissionStatus::EXECUTING ||
                last_execution_state_ == ugv_nav4d_ros2::msg::MissionStatus::PAUSED){
                response->success = false;
                response->message = "Route is still active (executing/paused); stop it before clearing the display.";
                publishStatus(response->message);
                return;
            }
            clearExecutingPathDisplay();
            response->success = true;
            response->message = "Executed-path display cleared.";
            publishStatus(response->message);
        });
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
    operational_zones_publisher = this->create_publisher<ugv_nav4d_ros2::msg::OperationalZoneArray>(
            "/ugv_nav4d_ros2/operational_zones", rclcpp::QoS(1).transient_local());

    delete_forbidden_zone_service = this->create_service<ugv_nav4d_ros2::srv::DeleteForbiddenZone>(
            "/ugv_nav4d_ros2/delete_forbidden_zone", std::bind(&PathPlannerNode::deleteForbiddenZoneCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Plan back to where the last mission started, visiting the waypoints in
    // reverse order.
    plan_return_service = this->create_service<std_srvs::srv::Trigger>(
            "/ugv_nav4d_ros2/plan_return", std::bind(&PathPlannerNode::planReturnCallback, this, std::placeholders::_1, std::placeholders::_2));
    plan_return_current_service = this->create_service<std_srvs::srv::Trigger>(
            "/ugv_nav4d_ros2/plan_return_current", std::bind(&PathPlannerNode::planReturnCurrentCallback, this, std::placeholders::_1, std::placeholders::_2));
    update_footprint_service = this->create_service<std_srvs::srv::Trigger>(
            "/ugv_nav4d_ros2/update_footprint", std::bind(&PathPlannerNode::updateFootprintCallback, this, std::placeholders::_1, std::placeholders::_2));
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
        publishStatus("Parameters changed; rebuilding maps and planner (this can take a while)...");
        configurePlanner();
        publishStatus("Planner reconfigured.");
    }
    const auto now = this->get_clock()->now();
    const size_t old_zone_count = forbidden_zones.size();
    const bool planning_zone_expired = std::any_of(
        forbidden_zones.begin(), forbidden_zones.end(),
        [&now](const auto& zone){
            const bool expired =
                (zone.expires_at.sec != 0 || zone.expires_at.nanosec != 0) &&
                rclcpp::Time(zone.expires_at) <= now;
            return expired && zoneAffectsPlanning(zone);
        });
    forbidden_zones.erase(
        std::remove_if(forbidden_zones.begin(), forbidden_zones.end(),
            [&now](const auto& zone){
                return (zone.expires_at.sec != 0 || zone.expires_at.nanosec != 0) &&
                       rclcpp::Time(zone.expires_at) <= now;
            }),
        forbidden_zones.end());
    if (forbidden_zones.size() != old_zone_count){
        publishStatus(std::to_string(old_zone_count - forbidden_zones.size()) +
                      " temporary operational zone(s) expired");
        onForbiddenZonesChanged(planning_zone_expired);
    }
}

void PathPlannerNode::poseUpdateTimerCallback(){
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
        groomUnderRobotTick();
    }
    geometry_msgs::msg::PoseStamped pose_message = start_pose;
    pose_message.header.stamp = this->get_clock()->now();
    pose_message.header.frame_id = get_parameter("world_frame").as_string();
    robot_pose_publisher->publish(pose_message);
    publishZoneSpeedLimit(pose_message.pose.position);
    if (pose_update_callback){
        pose_update_callback(pose);
    }
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
            // Grooming params are read live on every tick; no rebuild involved.
            if (param.get_name().rfind("groom_", 0) == 0)
            {
                continue;
            }
            // Footprint-update settings are only read when the service runs;
            // tuning them must not trigger a full planner rebuild by itself.
            if (param.get_name().rfind("footprint_", 0) == 0)
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
    // This full regeneration supersedes any rebuild queued by a parameter
    // change (e.g. recalibrate height); clearing the dirty flag prevents the
    // parameter timer from running a SECOND rebuild right after this one.
    parameters_to_update.clear();

    if (!get_parameter("read_pose_from_topic").as_bool() && !updatePoseFromTF())
    {
        response->success = false;
        response->message = "Cannot regenerate maps: TF lookup of the robot pose failed.";
        publishStatus(response->message);
        return;
    }

    has_pending_path = false;
    path_approved = false;
    publishPreviewPending();
    pending_path_is_recovery = false;
    std_msgs::msg::Bool route_valid;
    route_valid.data = false;
    route_valid_publisher->publish(route_valid);
    clearExecutingPathDisplay();

    ScopedRebuildFlag rebuild_flag(*this);
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
    const auto t_mls = std::chrono::steady_clock::now();
    got_map = generateMLS();
    RCLCPP_INFO_STREAM(this->get_logger(), "Timing: MLS merge of "
        << msg->width * msg->height << " points took "
        << std::chrono::duration<double>(std::chrono::steady_clock::now() - t_mls).count()
        << " s.");

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
            const auto t_expand = std::chrono::steady_clock::now();
            traversability_generator_ptr->expandAll(startPosition);
            RCLCPP_INFO_STREAM(this->get_logger(), "Timing: trav-map expansion took "
                << std::chrono::duration<double>(std::chrono::steady_clock::now() - t_expand).count()
                << " s.");
            applyForbiddenZones();
            rebuildSpeedZoneCache();
            auto travMap = traversability_generator_ptr->getTraversabilityMap();
            const bool first_env_build = !planner_ptr->isEnvironmentInitialized();
            const auto t_env = std::chrono::steady_clock::now();
            planner_ptr->updateMap(travMap);
            RCLCPP_INFO_STREAM(this->get_logger(), "Timing: "
                << (first_env_build
                    ? "environment build (incl. motion-primitive generation) took "
                    : "planner map update took ")
                << std::chrono::duration<double>(std::chrono::steady_clock::now() - t_env).count()
                << " s.");
            validatePendingPath();
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

void PathPlannerNode::autoPlanIfEnabled(){
    // Auto plan: the LAST queued waypoint acts as the goal, the earlier ones
    // are intermediate stops, and every queue change replans the whole chain
    // as a PREVIEW (execution still requires the Execute button). Planning
    // never consumes the queue, so each change replans the full chain.
    if (!auto_plan_enabled_ || waypoint_queue.empty()){
        return;
    }
    if (!got_map){
        publishStatus("Auto plan: no map yet; the chain plans once a map is loaded.");
        return;
    }
    if (is_planning){
        return;  // planning is synchronous; nothing sensible to do re-entrantly
    }
    if (!is_configured){
        configurePlanner();
    }
    if (!get_parameter("read_pose_from_topic").as_bool() && !updatePoseFromTF()){
        publishStatus("Auto plan: TF lookup of the robot pose failed; waypoint kept, not planned.");
        return;
    }
    start_pose_rbs.position = Eigen::Vector3d(start_pose.pose.position.x,
                                              start_pose.pose.position.y,
                                              start_pose.pose.position.z);
    start_pose_rbs.orientation = Eigen::Quaterniond(start_pose.pose.orientation.w,
                                                    start_pose.pose.orientation.x,
                                                    start_pose.pose.orientation.y,
                                                    start_pose.pose.orientation.z);
    // Move the tail into the goal slot for the duration of the plan, then
    // restore it: numbering, markers and pause-at-waypoints keep seeing the
    // full queue.
    const geometry_msgs::msg::Pose goal_waypoint = waypoint_queue.back();
    waypoint_queue.pop_back();
    goal_pose_rbs.position = Eigen::Vector3d(goal_waypoint.position.x,
                                             goal_waypoint.position.y,
                                             goal_waypoint.position.z);
    goal_pose_rbs.orientation = Eigen::Quaterniond(goal_waypoint.orientation.w,
                                                   goal_waypoint.orientation.x,
                                                   goal_waypoint.orientation.y,
                                                   goal_waypoint.orientation.z);
    if (waypoint_queue.empty()){
        plan();
    } else {
        planThroughWaypoints();
    }
    waypoint_queue.push_back(goal_waypoint);
}

void PathPlannerNode::autoPlanQueueChanged(){
    if (!auto_plan_enabled_){
        return;
    }
    if (waypoint_queue.empty()){
        // The preview was a chain through waypoints that no longer exist.
        if (has_pending_path){
            has_pending_path = false;
            path_approved = false;
            publishPreviewPending();
            pending_path_is_recovery = false;
            std_msgs::msg::Bool route_valid;
            route_valid.data = false;
            route_valid_publisher->publish(route_valid);
            clearExecutingPathDisplay();
            publishStatus("Auto plan: no waypoints left; preview discarded.");
        }
        return;
    }
    autoPlanIfEnabled();
}

void PathPlannerNode::addWaypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    waypoint_queue.push_back(msg->pose);
    RCLCPP_INFO_STREAM(this->get_logger(), "Added waypoint " << waypoint_queue.size()
                       << " at (" << msg->pose.position.x << ", " << msg->pose.position.y
                       << ", " << msg->pose.position.z << ")");
    publishWaypointMarkers();
    publishStatus(std::to_string(waypoint_queue.size()) + " waypoint(s) queued");
    autoPlanQueueChanged();
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
    autoPlanQueueChanged();
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
    autoPlanQueueChanged();
}

void PathPlannerNode::reverseWaypointsCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                               std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (waypoint_queue.empty()){
        response->success = false;
        response->message = "No waypoints queued.";
        return;
    }
    std::reverse(waypoint_queue.begin(), waypoint_queue.end());
    // Flip each heading half a turn: traversing the route the other way means
    // the robot passes every waypoint facing the opposite direction.
    const Eigen::Quaterniond half_turn(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
    for (auto& wp : waypoint_queue){
        Eigen::Quaterniond q(wp.orientation.w, wp.orientation.x, wp.orientation.y, wp.orientation.z);
        //A degenerate quaternion (all zeros / non-finite) would normalize to NaN
        //and poison the next plan with a NaN heading; treat it as identity.
        if (!q.coeffs().allFinite() || q.norm() < 1e-6){
            q = Eigen::Quaterniond::Identity();
        }
        q = (q * half_turn).normalized();
        wp.orientation.w = q.w();
        wp.orientation.x = q.x();
        wp.orientation.y = q.y();
        wp.orientation.z = q.z();
    }
    publishWaypointMarkers();
    response->success = true;
    response->message = "Reversed " + std::to_string(waypoint_queue.size()) +
                        " waypoint(s) and flipped their headings; set a goal and plan.";
    RCLCPP_INFO_STREAM(this->get_logger(), response->message);
    publishStatus(response->message);
    autoPlanQueueChanged();
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
    if (request->truncate_after){
        const size_t removed = waypoint_queue.size() - request->index;
        if (removed == 0){
            response->success = true;
            response->message = "No waypoints after " + std::to_string(request->index)
                                + "; queue unchanged.";
            publishStatus(response->message);
            return;
        }
        waypoint_queue.erase(waypoint_queue.begin() + request->index, waypoint_queue.end());
        response->message = "Removed " + std::to_string(removed) + " waypoint(s) after "
                            + std::to_string(request->index) + "; "
                            + std::to_string(waypoint_queue.size()) + " remaining.";
    } else if (request->truncate_before){
        const size_t removed = i;
        if (removed == 0){
            response->success = true;
            response->message = "No waypoints before " + std::to_string(request->index)
                                + "; queue unchanged.";
            publishStatus(response->message);
            return;
        }
        waypoint_queue.erase(waypoint_queue.begin(), waypoint_queue.begin() + i);
        response->message = "Removed " + std::to_string(removed) + " waypoint(s) before "
                            + std::to_string(request->index) + "; "
                            + std::to_string(waypoint_queue.size())
                            + " remaining (renumbered).";
    } else if (request->remove){
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
    autoPlanQueueChanged();
}

void PathPlannerNode::publishWaypointMarkers(){
    // Latched queue snapshot for the follower (pause-at-waypoints) and the
    // operator panel (toggle enable state). Markers below are display-only.
    geometry_msgs::msg::PoseArray queue_msg;
    queue_msg.header.frame_id = get_parameter("world_frame").as_string();
    queue_msg.header.stamp = this->get_clock()->now();
    queue_msg.poses.assign(waypoint_queue.begin(), waypoint_queue.end());
    waypoint_poses_publisher->publish(queue_msg);

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
        // Floating well above the arrows by operator preference: airborne
        // labels stay readable over terrain relief and marker clutter.
        text.pose.position.z += 2.5 - dist_to_ground;
        text.scale.z = 1.0;
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

    // Anchor recording is deferred to executePathCallback (see plan()).
    pending_records_mission = record_mission;
    if (record_mission){
        pending_mission_start = start_pose_rbs;
        pending_mission_goal = goal_pose_rbs;
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
            // fix the offending waypoint and retry. The status text names the offending
            // POSE and the likely cause: e.g. leg 4's start is waypoint 3, which can be
            // a valid goal (generous goal margins) yet an invalid start (exact heading
            // must be allowed on that cell).
            const std::string start_name = (k == 0)
                ? std::string("the robot's current pose")
                : "waypoint " + std::to_string(k);
            std::ostringstream why;
            why.setf(std::ios::fixed);
            why.precision(1);
            why << "Waypoint leg " << (k + 1) << "/" << targets.size() << " failed ("
                << planningResultToString(res) << "): ";
            switch (res){
            case ugv_nav4d::Planner::START_INVALID:
                why << start_name << " at (" << segment_start.position.x() << ", "
                    << segment_start.position.y() << ") is not a valid START: no traversable "
                    << "cell within " << get_parameter("searchRadius").as_double()
                    << " m allows its exact heading (obstacle, disallowed orientation on a "
                    << "partially traversable cell, or height mismatch). "
                    << (k == 0
                        ? std::string("Replan after moving the robot to open ground, or regenerate the map.")
                        : "Move or rotate waypoint " + std::to_string(k)
                          + " onto open ground (check it with Inspect Traversability), then replan.");
                break;
            case ugv_nav4d::Planner::GOAL_INVALID:
                why << "waypoint " << (k + 1) << " at (" << targets[k].position.x() << ", "
                    << targets[k].position.y() << ") is not a valid GOAL even within the goal "
                    << "margins. Move or rotate waypoint " << (k + 1)
                    << " onto traversable ground, then replan.";
                break;
            case ugv_nav4d::Planner::NO_SOLUTION:
                why << "no path found from " << start_name << " to waypoint " << (k + 1)
                    << " with the current map and footprint (blocked or too-narrow corridor, "
                    << "or maxTime too low). Check the trav map between them.";
                break;
            case ugv_nav4d::Planner::NO_MAP:
                why << "no traversability map is available; load or regenerate the map first.";
                break;
            default:
                why << "internal planner error between " << start_name << " and waypoint "
                    << (k + 1) << "; see the planner log.";
                break;
            }
            why << " Mission aborted; the waypoint queue is kept for correction.";
            RCLCPP_ERROR_STREAM(this->get_logger(), why.str());
            publishStatus(why.str());
            latest_trajectory2D.clear();
            latest_trajectory3D.clear();
            latest_planning_result = res;
            publishPlannedPath({}, false);
            return;
        }

        combined2D.insert(combined2D.end(), trajectory2D.begin(), trajectory2D.end());
        combined3D.insert(combined3D.end(), trajectory3D.begin(), trajectory3D.end());
        // Chain the next leg from where this leg ACTUALLY ends, not from the
        // ideal waypoint pose: plan() is allowed to stop anywhere within
        // goalDistanceMargin/goalOrientationMargin of the waypoint, so
        // restarting at the waypoint itself produced visible (and driven)
        // jumps of up to those margins at every waypoint. The achieved end
        // pose is also a validated planner state, so it is always a legal
        // start (the ideal waypoint pose sometimes was not -> START_INVALID).
        if (!trajectory3D.empty()){
            const auto& leg_end = trajectory3D.back();
            double end_yaw = leg_end.goalPose.orientation;
            // Ackermann spline headings follow the travel direction; a leg
            // that ends reversing has the body facing the opposite way.
            if (leg_end.driveMode == trajectory_follower::DriveMode::ModeAckermann &&
                leg_end.speed < 0){
                end_yaw += M_PI;
            }
            segment_start = targets[k];  // keeps the waypoint's body-frame z
            segment_start.position.x() = leg_end.goalPose.position.x();
            segment_start.position.y() = leg_end.goalPose.position.y();
            segment_start.orientation = Eigen::Quaterniond(
                Eigen::AngleAxisd(end_yaw, Eigen::Vector3d::UnitZ()));
        } else {
            segment_start = targets[k];
        }
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

void PathPlannerNode::publishPreviewPending(){
    std_msgs::msg::Bool msg;
    msg.data = has_pending_path && !path_approved;
    preview_pending_publisher->publish(msg);
}

void PathPlannerNode::publishStatus(const std::string& status){
    std_msgs::msg::String msg;
    msg.data = status;
    status_publisher->publish(msg);
    if (status_callback) {
        status_callback(status);
    }

    std::string lower = status;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    uint8_t state = ugv_nav4d_ros2::msg::MissionStatus::READY;
    std::string failure;
    if (lower.find("planning") != std::string::npos){
        state = ugv_nav4d_ros2::msg::MissionStatus::PLANNING;
    } else if (lower.find("path ready") != std::string::npos){
        state = ugv_nav4d_ros2::msg::MissionStatus::AWAITING_APPROVAL;
    } else if (lower.find("sent to follower") != std::string::npos){
        state = ugv_nav4d_ros2::msg::MissionStatus::EXECUTING;
    } else if (lower.find("failed") != std::string::npos ||
               lower.find("cannot") != std::string::npos ||
               lower.find("no map") != std::string::npos){
        state = ugv_nav4d_ros2::msg::MissionStatus::FAILED;
        failure = status;
    }
    publishMissionStatus(state, status, failure);
}

void PathPlannerNode::publishMissionStatus(uint8_t state, const std::string& summary,
                                           const std::string& failure_reason){
    if (!mission_status_publisher){
        return;
    }
    ugv_nav4d_ros2::msg::MissionStatus msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = get_parameter("world_frame").as_string();
    msg.state = state;
    switch (state){
        case ugv_nav4d_ros2::msg::MissionStatus::READY: msg.state_name = "READY"; break;
        case ugv_nav4d_ros2::msg::MissionStatus::PLANNING: msg.state_name = "PLANNING"; break;
        case ugv_nav4d_ros2::msg::MissionStatus::AWAITING_APPROVAL: msg.state_name = "AWAITING_APPROVAL"; break;
        case ugv_nav4d_ros2::msg::MissionStatus::EXECUTING: msg.state_name = "EXECUTING"; break;
        case ugv_nav4d_ros2::msg::MissionStatus::PAUSED: msg.state_name = "PAUSED"; break;
        case ugv_nav4d_ros2::msg::MissionStatus::COMPLETED: msg.state_name = "COMPLETED"; break;
        case ugv_nav4d_ros2::msg::MissionStatus::FAILED: msg.state_name = "FAILED"; break;
        case ugv_nav4d_ros2::msg::MissionStatus::ABORTED: msg.state_name = "ABORTED"; break;
        default: msg.state_name = "IDLE"; break;
    }
    msg.summary = summary;
    msg.failure_reason = failure_reason;
    msg.total_segments = static_cast<uint32_t>(pending_labeled_path.paths.size());
    msg.route_valid = has_pending_path;
    mission_status_publisher->publish(msg);
}

static std::string nodeTypeName(traversability_generator3d::NodeType type){
    std::ostringstream stream;
    stream << type;
    return stream.str();
}

static std::string obstacleCauseName(traversability_generator3d::ObstacleCause cause){
    using traversability_generator3d::ObstacleCause;
    switch(cause){
        case ObstacleCause::NONE:          return "NONE";
        case ObstacleCause::UNMEASURED:    return "UNMEASURED";
        case ObstacleCause::STEEP_SLOPE:   return "STEEP_SLOPE";
        case ObstacleCause::STEP_HEIGHT:   return "STEP_HEIGHT";
        case ObstacleCause::INCLINE_LIMIT: return "INCLINE_LIMIT";
        case ObstacleCause::NO_SAFE_YAW:   return "NO_SAFE_YAW";
        case ObstacleCause::MAP_BOUNDARY:  return "MAP_BOUNDARY";
    }
    return "UNKNOWN";
}

void PathPlannerNode::inspectTraversabilityCallback(
        const std::shared_ptr<ugv_nav4d_ros2::srv::InspectTraversability::Request> request,
        std::shared_ptr<ugv_nav4d_ros2::srv::InspectTraversability::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (!traversability_generator_ptr){
        response->success = false;
        response->message = "Traversability map is unavailable.";
        return;
    }
    const auto& map = traversability_generator_ptr->getTraversabilityMap();
    maps::grid::Index index;
    const Eigen::Vector3d query(request->point.x, request->point.y, request->point.z);
    if (!map.toGrid(query, index)){
        response->success = false;
        response->message = "Point is outside the traversability map.";
        return;
    }
    const traversability_generator3d::TravGenNode* best = nullptr;
    double best_distance = std::numeric_limits<double>::max();
    Eigen::Vector3d best_position;
    for (const auto* node : map.at(index)){
        Eigen::Vector3d position;
        if (!map.fromGrid(node->getIndex(), position, node->getHeight(), false)){
            continue;
        }
        const double distance = (position - query).norm();
        if (distance < best_distance){
            best = node;
            best_distance = distance;
            best_position = position;
        }
    }
    if (!best){
        response->success = false;
        response->message = "No traversability surface exists in this cell.";
        return;
    }
    const auto& data = best->getUserData();
    response->success = true;
    response->message = nodeTypeName(data.nodeType);
    response->matched_point.x = best_position.x();
    response->matched_point.y = best_position.y();
    response->matched_point.z = best_position.z();
    response->distance = best_distance;
    response->type = static_cast<uint8_t>(data.nodeType);
    response->type_name = nodeTypeName(data.nodeType);
    response->slope = data.slope;
    response->slope_direction = data.slopeDirectionAtan2;
    response->cost = data.cost;
    for (const auto& allowed : data.allowedOrientations){
        response->allowed_orientation_starts.push_back(allowed.getStart().getRad());
        response->allowed_orientation_widths.push_back(allowed.getWidth());
    }
}

void PathPlannerNode::saveMapCallback(const std::shared_ptr<ugv_nav4d_ros2::srv::MissionFile::Request> request,
                                      std::shared_ptr<ugv_nav4d_ros2::srv::MissionFile::Response> response){
    if (!got_map || !mls_map_ptr){
        response->success = false;
        response->message = "No MLS map available to save.";
        publishStatus(response->message);
        return;
    }
    //Full path from the caller (panel file dialog); empty keeps the legacy
    //behavior of a timestamped file in the node working directory.
    const std::string filename = request->filename.empty()
        ? generateTimestampedFilename(".bin")
        : request->filename;
    if (saveMLSMapAsBin(filename)){
        response->success = true;
        response->message = "MLS map saved to " + filename;
    } else {
        response->success = false;
        response->message = "Failed to save MLS map to " + filename;
    }
    publishStatus(response->message);
}

void PathPlannerNode::replanCurrentMissionCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (!got_map || !is_configured){
        response->success = false;
        response->message = "Cannot replan: planner or map is unavailable.";
        return;
    }
    if (!get_parameter("read_pose_from_topic").as_bool() && !updatePoseFromTF()){
        response->success = false;
        response->message = "Cannot replan: current robot pose is unavailable.";
        return;
    }
    start_pose_rbs.position = Eigen::Vector3d(
        start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z);
    start_pose_rbs.orientation = Eigen::Quaterniond(
        start_pose.pose.orientation.w, start_pose.pose.orientation.x,
        start_pose.pose.orientation.y, start_pose.pose.orientation.z);
    if (waypoint_queue.empty()) plan(false);
    else planThroughWaypoints(false);
    response->success = latest_planning_result == ugv_nav4d::Planner::FOUND_SOLUTION;
    response->message = response->success
        ? "Replanned from the current robot pose; review the new path before Execute."
        : std::string("Replanning failed: ") + planningResultToString(latest_planning_result);
    publishStatus(response->message);
}

void PathPlannerNode::recoverOutOfObstacleCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (!got_map || !is_configured || !planner_ptr){
        response->success = false;
        response->message = "Cannot recover: planner or traversability map is unavailable.";
        publishStatus(response->message);
        return;
    }
    if (!get_parameter("read_pose_from_topic").as_bool() && !updatePoseFromTF()){
        response->success = false;
        response->message = "Cannot recover: current robot pose is unavailable.";
        publishStatus(response->message);
        return;
    }

    Eigen::Affine3d body_to_world(Eigen::Affine3d::Identity());
    body_to_world.translation() = Eigen::Vector3d(
        start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z);
    const Eigen::Quaterniond body_orientation(
        start_pose.pose.orientation.w, start_pose.pose.orientation.x,
        start_pose.pose.orientation.y, start_pose.pose.orientation.z);
    if (!body_orientation.coeffs().allFinite() || body_orientation.norm() < 1e-6){
        response->success = false;
        response->message = "Cannot recover: current robot orientation is invalid.";
        publishStatus(response->message);
        return;
    }
    body_to_world.linear() = body_orientation.normalized().toRotationMatrix();

    // The native ugv_nav4d recovery API operates from the ground-contact pose
    // and returns the trajectory in the body-height convention used elsewhere.
    Eigen::Affine3d ground_to_body(Eigen::Affine3d::Identity());
    ground_to_body.translation() = Eigen::Vector3d(
        0.0, 0.0, -get_parameter("distToGround").as_double());
    const Eigen::Affine3d ground_to_world = body_to_world * ground_to_body;
    const double yaw = base::getYaw(body_orientation);

    const auto rescue = planner_ptr->findTrajectoryOutOfObstacle(
        ground_to_world.translation(), yaw, ground_to_body, false);
    if (!rescue || rescue->posSpline.getEndParam() <= rescue->posSpline.getStartParam()){
        response->success = false;
        response->message = "Native recovery found no motion that ends outside obstacles/frontiers.";
        publishPlannedPath({}, false);
        publishStatus(response->message);
        return;
    }

    latest_trajectory3D = {*rescue};
    latest_trajectory2D.clear();
    publishPlannedPath(latest_trajectory3D, true);
    response->success = true;
    response->message = "Native recovery trajectory is ready for review; press Execute path to send it.";
    publishStatus(response->message);
}

void PathPlannerNode::saveMissionCallback(
        const std::shared_ptr<ugv_nav4d_ros2::srv::MissionFile::Request> request,
        std::shared_ptr<ugv_nav4d_ros2::srv::MissionFile::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    const std::string filename = request->filename.empty()
        ? get_parameter("mission_file").as_string()
        : request->filename;
    std::ofstream out(filename);
    if (!out){
        response->success = false;
        response->message = "Cannot open mission file for writing: " + filename;
        return;
    }
    // V2 stores waypoints and zones only. The goal is deliberately NOT part of a
    // mission: it is set fresh per run (RViz / OperatorPanel), so a reloaded
    // mission can never silently drive to yesterday's goal.
    out << "UGV_NAV4D_MISSION_V2\n";
    out << std::setprecision(17);
    out << "waypoints " << waypoint_queue.size() << '\n';
    for (const auto& wp : waypoint_queue){
        out << "wp " << wp.position.x << ' ' << wp.position.y << ' ' << wp.position.z << ' '
            << wp.orientation.x << ' ' << wp.orientation.y << ' '
            << wp.orientation.z << ' ' << wp.orientation.w << '\n';
    }
    out << "zones " << forbidden_zones.size() << '\n';
    for (const auto& zone : forbidden_zones){
        out << "zone " << static_cast<unsigned>(zone.zone_type) << ' '
            << std::quoted(zone.label) << ' ' << zone.cost_multiplier << ' '
            << zone.speed_limit << ' ' << zone.preferred_heading << ' '
            << zone.expires_at.sec << ' ' << zone.expires_at.nanosec << ' '
            << zone.vertices.size() << '\n';
        if (zone.zone_type == ugv_nav4d_ros2::msg::ForbiddenZone::TRAVERSABLE){
            // Optional line; loaders older than the TRAVERSABLE type stop at
            // unknown tokens, and such zones did not exist in old files.
            out << "fill " << (zone.fill_enabled ? 1 : 0) << ' ' << zone.fill_z_offset
                << ' ' << zone.fill_roll << ' ' << zone.fill_pitch
                << ' ' << zone.delete_top << '\n';
        }
        for (const auto& v : zone.vertices) out << "v " << v.x << ' ' << v.y << ' ' << v.z << '\n';
    }
    // Optional trailing section: photos captured at waypoint pauses during the
    // last execution (JSON: waypoint number -> file path on the robot). Old
    // loaders stop after the zones and never see it.
    if (!waypoint_photos_json.empty() && waypoint_photos_json != "{}"){
        out << "photos " << std::quoted(waypoint_photos_json) << '\n';
    }
    response->success = static_cast<bool>(out);
    response->message = response->success ? "Mission saved to " + filename
                                          : "Failed while writing mission file " + filename;
    publishStatus(response->message);
}

void PathPlannerNode::loadMissionCallback(
        const std::shared_ptr<ugv_nav4d_ros2::srv::MissionFile::Request> request,
        std::shared_ptr<ugv_nav4d_ros2::srv::MissionFile::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    const std::string filename = request->filename.empty()
        ? get_parameter("mission_file").as_string()
        : request->filename;
    std::ifstream in(filename);
    std::string token, magic;
    if (!(in >> magic) || (magic != "UGV_NAV4D_MISSION_V1" && magic != "UGV_NAV4D_MISSION_V2")){
        response->success = false;
        response->message = "Invalid or unreadable mission file: " + filename;
        return;
    }
    if (magic == "UGV_NAV4D_MISSION_V1"){
        // V1 files stored a goal pose; parse and DISCARD it. Missions carry only
        // waypoints and zones — the goal is always set fresh by the operator.
        base::samples::RigidBodyState ignored_goal;
        if (!(in >> token) || token != "goal" ||
            !(in >> ignored_goal.position.x() >> ignored_goal.position.y() >> ignored_goal.position.z()
                 >> ignored_goal.orientation.x() >> ignored_goal.orientation.y()
                 >> ignored_goal.orientation.z() >> ignored_goal.orientation.w())){
            response->success = false;
            response->message = "Mission file has an invalid goal.";
            return;
        }
    }
    size_t waypoint_count = 0;
    if (!(in >> token >> waypoint_count) || token != "waypoints"){
        response->success = false;
        response->message = "Mission file has an invalid waypoint section.";
        return;
    }
    std::vector<geometry_msgs::msg::Pose> loaded_waypoints;
    for (size_t i = 0; i < waypoint_count; ++i){
        geometry_msgs::msg::Pose wp;
        if (!(in >> token) || token != "wp" ||
            !(in >> wp.position.x >> wp.position.y >> wp.position.z
                 >> wp.orientation.x >> wp.orientation.y >> wp.orientation.z >> wp.orientation.w)){
            response->success = false;
            response->message = "Mission file has an invalid waypoint.";
            return;
        }
        //Reject degenerate orientations (a zero quaternion normalizes to NaN and
        //would poison later planning); normalize slightly-off ones.
        const double quat_norm = std::sqrt(
            wp.orientation.x * wp.orientation.x + wp.orientation.y * wp.orientation.y +
            wp.orientation.z * wp.orientation.z + wp.orientation.w * wp.orientation.w);
        if (!std::isfinite(quat_norm) || quat_norm < 1e-6 ||
            !std::isfinite(wp.position.x + wp.position.y + wp.position.z)){
            response->success = false;
            response->message = "Mission file has a waypoint with an invalid pose.";
            return;
        }
        wp.orientation.x /= quat_norm;
        wp.orientation.y /= quat_norm;
        wp.orientation.z /= quat_norm;
        wp.orientation.w /= quat_norm;
        loaded_waypoints.push_back(wp);
    }
    size_t zone_count = 0;
    if (!(in >> token >> zone_count) || token != "zones"){
        response->success = false;
        response->message = "Mission file has an invalid zone section.";
        return;
    }
    std::vector<ugv_nav4d_ros2::msg::ForbiddenZone> loaded_zones;
    for (size_t i = 0; i < zone_count; ++i){
        ugv_nav4d_ros2::msg::ForbiddenZone zone;
        unsigned zone_type = 0;
        size_t vertex_count = 0;
        if (!(in >> token) || token != "zone" ||
            !(in >> zone_type >> std::quoted(zone.label) >> zone.cost_multiplier
                 >> zone.speed_limit >> zone.preferred_heading
                 >> zone.expires_at.sec >> zone.expires_at.nanosec >> vertex_count)){
            response->success = false;
            response->message = "Mission file has invalid zone metadata.";
            return;
        }
        zone.zone_type = static_cast<uint8_t>(zone_type);
        zone.header.frame_id = get_parameter("world_frame").as_string();
        if (zone.zone_type == ugv_nav4d_ros2::msg::ForbiddenZone::TRAVERSABLE &&
            (in >> std::ws) && in.peek() == 'f'){
            int fill_enabled = 0;
            if (!(in >> token) || token != "fill" ||
                !(in >> fill_enabled >> zone.fill_z_offset >> zone.fill_roll
                     >> zone.fill_pitch >> zone.delete_top)){
                response->success = false;
                response->message = "Mission file has an invalid zone fill line.";
                return;
            }
            zone.fill_enabled = fill_enabled != 0;
        }
        for (size_t k = 0; k < vertex_count; ++k){
            geometry_msgs::msg::Point v;
            if (!(in >> token) || token != "v" || !(in >> v.x >> v.y >> v.z)){
                response->success = false;
                response->message = "Mission file has an invalid zone vertex.";
                return;
            }
            zone.vertices.push_back(v);
        }
        if (zone.vertices.size() < 3){
            response->success = false;
            response->message = "Mission file contains an operational zone with fewer than three vertices.";
            return;
        }
        double twice_area = 0.0;
        bool finite_vertices = true;
        for (size_t k = 0, j = zone.vertices.size() - 1;
             k < zone.vertices.size(); j = k++){
            const auto& a = zone.vertices[j];
            const auto& b = zone.vertices[k];
            finite_vertices = finite_vertices && std::isfinite(a.x) &&
                std::isfinite(a.y) && std::isfinite(a.z);
            twice_area += a.x * b.y - b.x * a.y;
        }
        const bool valid_speed =
            zone.zone_type != ugv_nav4d_ros2::msg::ForbiddenZone::SPEED_LIMIT ||
            (std::isfinite(zone.speed_limit) && zone.speed_limit > 0.0f);
        if (!finite_vertices || std::abs(twice_area) <= 1e-6 || !valid_speed){
            response->success = false;
            response->message = "Mission file contains an invalid operational zone.";
            return;
        }
        loaded_zones.push_back(zone);
    }
    waypoint_queue = std::move(loaded_waypoints);
    const bool planning_graph_changed =
        std::any_of(forbidden_zones.begin(), forbidden_zones.end(), zoneAffectsPlanning) ||
        std::any_of(loaded_zones.begin(), loaded_zones.end(), zoneAffectsPlanning);
    forbidden_zones = std::move(loaded_zones);
    std::string photo_note;
    if ((in >> token) && token == "photos"){
        std::string photos_json;
        if (in >> std::quoted(photos_json) && !photos_json.empty()){
            waypoint_photos_json = photos_json;
            const size_t count = static_cast<size_t>(
                std::count(photos_json.begin(), photos_json.end(), ':'));
            photo_note = " Mission references " + std::to_string(count) +
                         " waypoint photo(s); paths logged.";
            RCLCPP_INFO_STREAM(this->get_logger(),
                "Waypoint photos from mission file: " << photos_json);
        }
    }
    publishWaypointMarkers();
    onForbiddenZonesChanged(planning_graph_changed);
    response->success = true;
    response->message = "Mission loaded from " + filename +
                        " (waypoints + zones); set a goal and plan." + photo_note;
    publishStatus(response->message);
    // After zones are applied, so an auto-plan preview uses the updated map.
    autoPlanQueueChanged();
}

// Even-odd rule point-in-polygon test in the XY plane.
// Zones are drawn on a map surface, so their vertex z values carry the level
// they were drawn on. A zone applies only within [min vz - margin, max vz +
// margin]; the vertex spread absorbs slopes inside large polygons. Legacy or
// synthetic zones with all-zero vertex z keep the old whole-column behavior.
static bool zoneLevelBandContains(const std::vector<geometry_msgs::msg::Point>& vertices, double z){
    constexpr double kLevelMargin = 2.0;
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    bool any_nonzero = false;
    for (const auto& v : vertices){
        min_z = std::min(min_z, v.z);
        max_z = std::max(max_z, v.z);
        if (std::abs(v.z) > 1e-9){
            any_nonzero = true;
        }
    }
    if (!any_nonzero || vertices.empty()){
        return true; // whole column (legacy zones without height information)
    }
    return z >= min_z - kLevelMargin && z <= max_z + kLevelMargin;
}

static bool pointInPolygonXY(double x, double y, const std::vector<geometry_msgs::msg::Point>& poly){
    if (poly.size() < 3){
        return false;
    }
    // Treat the boundary as inside. This prevents the speed command oscillating
    // when localization noise places the robot on alternating sides of an edge.
    constexpr double boundary_epsilon = 1e-6;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++){
        const double ab_x = poly[i].x - poly[j].x;
        const double ab_y = poly[i].y - poly[j].y;
        const double ap_x = x - poly[j].x;
        const double ap_y = y - poly[j].y;
        const double cross = std::abs(ab_x * ap_y - ab_y * ap_x);
        const double edge_length = std::hypot(ab_x, ab_y);
        const double dot = ap_x * ab_x + ap_y * ab_y;
        if (cross <= boundary_epsilon * std::max(1.0, edge_length) &&
            dot >= -boundary_epsilon && dot <= ab_x * ab_x + ab_y * ab_y + boundary_epsilon){
            return true;
        }
    }
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

void PathPlannerNode::rebuildSpeedZoneCache(){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    speed_zone_node_sets.clear();
    speed_zone_node_sets.resize(forbidden_zones.size());
    if (!traversability_generator_ptr){
        return;
    }

    const auto& map = traversability_generator_ptr->getTraversabilityMap();
    const auto now = this->get_clock()->now();
    const double surface_tolerance = std::max(
        0.0, get_parameter("speed_zone_surface_tolerance").as_double());
    for (size_t zone_index = 0; zone_index < forbidden_zones.size(); ++zone_index){
        const auto& zone = forbidden_zones[zone_index];
        const bool expired =
            (zone.expires_at.sec != 0 || zone.expires_at.nanosec != 0) &&
            rclcpp::Time(zone.expires_at) <= now;
        if (expired || zone.zone_type != ugv_nav4d_ros2::msg::ForbiddenZone::SPEED_LIMIT ||
            zone.vertices.size() < 3){
            continue;
        }

        auto& zone_nodes = speed_zone_node_sets[zone_index];
        std::vector<traversability_generator3d::TravGenNode*> seeds;
        std::deque<traversability_generator3d::TravGenNode*> queue;
        for (const auto& vertex : zone.vertices){
            maps::grid::Index index;
            if (!map.toGrid(Eigen::Vector3d(vertex.x, vertex.y, vertex.z), index)){
                continue;
            }
            traversability_generator3d::TravGenNode* seed = nullptr;
            double best_dz = std::numeric_limits<double>::max();
            for (auto* node : map.at(index)){
                const auto type = node->getUserData().nodeType;
                if (type != traversability_generator3d::NodeType::TRAVERSABLE &&
                    type != traversability_generator3d::NodeType::PARTIALLY_TRAVERSABLE){
                    continue;
                }
                const double dz = std::abs(node->getHeight() - vertex.z);
                if (dz < best_dz){
                    best_dz = dz;
                    seed = node;
                }
            }
            if (seed && best_dz <= surface_tolerance){
                seeds.push_back(seed);
            }
        }

        if (seeds.empty()){
            RCLCPP_WARN_STREAM(this->get_logger(), "Speed-limit zone " << (zone_index + 1)
                               << " matched no traversability surface; the conservative XY fallback will be used.");
            continue;
        }
        zone_nodes.insert(seeds.front());
        queue.push_back(seeds.front());

        // Restrict the flood-fill to the polygon. Even if two storeys are globally
        // connected by a ramp, an overlapping XY footprint cannot jump vertically
        // unless that connection itself lies inside this zone.
        while (!queue.empty()){
            auto* node = queue.front();
            queue.pop_front();
            for (auto* base_neighbor : node->getConnections()){
                auto* neighbor = static_cast<traversability_generator3d::TravGenNode*>(base_neighbor);
                if (zone_nodes.count(neighbor)){
                    continue;
                }
                const auto type = neighbor->getUserData().nodeType;
                if (type != traversability_generator3d::NodeType::TRAVERSABLE &&
                    type != traversability_generator3d::NodeType::PARTIALLY_TRAVERSABLE){
                    continue;
                }
                Eigen::Vector3d position;
                if (!map.fromGrid(neighbor->getIndex(), position, neighbor->getHeight(), false) ||
                    !pointInPolygonXY(position.x(), position.y(), zone.vertices) ||
                    !zoneLevelBandContains(zone.vertices, position.z())){
                    // The band stops the one remaining vertical path: a ramp
                    // that lies inside the polygon and connects the storeys.
                    continue;
                }
                zone_nodes.insert(neighbor);
                queue.push_back(neighbor);
            }
        }
        const bool ambiguous_surface = std::any_of(
            seeds.begin(), seeds.end(),
            [&zone_nodes](const auto* seed){ return !zone_nodes.count(seed); });
        if (ambiguous_surface){
            zone_nodes.clear();
            RCLCPP_WARN_STREAM(this->get_logger(), "Speed-limit zone " << (zone_index + 1)
                               << " has vertices on disconnected surfaces; the conservative XY fallback will be used.");
        }
    }
}

void PathPlannerNode::emitZoneSpeedLimit(float limit){
    const auto now = this->get_clock()->now();
    if (!std::isfinite(limit) || limit < 0.0f){
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Refusing to publish an invalid zone speed limit.");
        limit = last_zone_speed_limit;
    }
    if (limit > 0.0f){
        last_zone_speed_limit = limit;
        last_zone_speed_limit_match = now;
    } else {
        const double release_delay = std::max(
            0.0, get_parameter("speed_zone_release_delay").as_double());
        if (last_zone_speed_limit > 0.0f &&
            (now - last_zone_speed_limit_match).seconds() < release_delay){
            limit = last_zone_speed_limit;
        } else {
            last_zone_speed_limit = 0.0f;
        }
    }
    std_msgs::msg::Float32 result;
    result.data = limit;
    zone_speed_limit_publisher->publish(result);
}

void PathPlannerNode::publishZoneSpeedLimit(const geometry_msgs::msg::Point& robot_position){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    const auto now = this->get_clock()->now();

    // If surface matching becomes unavailable, apply the lowest XY-overlapping
    // limit. Slowing on an extra storey is safer than silently dropping a limit.
    float xy_fallback_limit = 0.0f;
    for (const auto& zone : forbidden_zones){
        const bool expired =
            (zone.expires_at.sec != 0 || zone.expires_at.nanosec != 0) &&
            rclcpp::Time(zone.expires_at) <= now;
        if (!expired && zone.zone_type == ugv_nav4d_ros2::msg::ForbiddenZone::SPEED_LIMIT &&
            std::isfinite(zone.speed_limit) && zone.speed_limit > 0.0f &&
            pointInPolygonXY(robot_position.x, robot_position.y, zone.vertices) &&
            zoneLevelBandContains(zone.vertices,
                robot_position.z - get_parameter("distToGround").as_double())){
            xy_fallback_limit = xy_fallback_limit > 0.0f
                ? std::min(xy_fallback_limit, zone.speed_limit)
                : zone.speed_limit;
        }
    }

    if (!got_map || !is_configured || !traversability_generator_ptr ||
        speed_zone_node_sets.size() != forbidden_zones.size()){
        emitZoneSpeedLimit(xy_fallback_limit);
        return;
    }

    const auto& map = traversability_generator_ptr->getTraversabilityMap();
    maps::grid::Index robot_index;
    const double ground_z = robot_position.z - get_parameter("distToGround").as_double();
    if (!map.toGrid(Eigen::Vector3d(robot_position.x, robot_position.y, ground_z), robot_index)){
        emitZoneSpeedLimit(xy_fallback_limit);
        return;
    }

    const traversability_generator3d::TravGenNode* robot_node = nullptr;
    double best_dz = std::numeric_limits<double>::max();
    for (const auto* node : map.at(robot_index)){
        const auto type = node->getUserData().nodeType;
        if (type != traversability_generator3d::NodeType::TRAVERSABLE &&
            type != traversability_generator3d::NodeType::PARTIALLY_TRAVERSABLE){
            continue;
        }
        const double dz = std::abs(node->getHeight() - ground_z);
        if (dz < best_dz){
            best_dz = dz;
            robot_node = node;
        }
    }

    const double surface_tolerance = std::max(
        0.0, get_parameter("speed_zone_surface_tolerance").as_double());
    if (!robot_node || best_dz > surface_tolerance){
        if (xy_fallback_limit > 0.0f){
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Robot pose did not match a nearby traversability surface; applying the fail-safe XY speed limit.");
        }
        emitZoneSpeedLimit(xy_fallback_limit);
        return;
    }

    float resolved_limit = 0.0f;
    for (size_t i = 0; robot_node && i < forbidden_zones.size(); ++i){
        const auto& zone = forbidden_zones[i];
        const bool expired =
            (zone.expires_at.sec != 0 || zone.expires_at.nanosec != 0) &&
            rclcpp::Time(zone.expires_at) <= now;
        if (expired || zone.zone_type != ugv_nav4d_ros2::msg::ForbiddenZone::SPEED_LIMIT ||
            !std::isfinite(zone.speed_limit) || zone.speed_limit <= 0.0f ||
            !pointInPolygonXY(robot_position.x, robot_position.y, zone.vertices) ||
            !zoneLevelBandContains(zone.vertices, robot_node->getHeight())){
            continue;
        }
        // An empty cache means the drawn surface could not be resolved. Apply
        // this zone conservatively at matching XY until the cache can be rebuilt.
        if (!speed_zone_node_sets[i].empty() && !speed_zone_node_sets[i].count(robot_node)){
            continue;
        }
        resolved_limit = resolved_limit > 0.0f
            ? std::min(resolved_limit, zone.speed_limit)
            : zone.speed_limit;
    }
    emitZoneSpeedLimit(resolved_limit);
}

void PathPlannerNode::addForbiddenZoneCallback(const ugv_nav4d_ros2::msg::ForbiddenZone::SharedPtr msg){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (msg->vertices.size() < 3){
        RCLCPP_WARN_STREAM(this->get_logger(), "Ignoring operational zone with only "
                           << msg->vertices.size() << " vertices (minimum 3).");
        return;
    }
    const std::string world_frame = normalizedFrame(get_parameter("world_frame").as_string());
    const std::string zone_frame = normalizedFrame(msg->header.frame_id);
    if (!zone_frame.empty() && zone_frame != world_frame){
        RCLCPP_ERROR_STREAM(this->get_logger(), "Ignoring operational zone in frame '"
                            << msg->header.frame_id << "'; expected '" << world_frame << "'.");
        publishStatus("Operational zone rejected: RViz fixed frame must match " + world_frame);
        return;
    }
    double twice_area = 0.0;
    for (size_t i = 0, j = msg->vertices.size() - 1; i < msg->vertices.size(); j = i++){
        const auto& a = msg->vertices[j];
        const auto& b = msg->vertices[i];
        if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z)){
            RCLCPP_ERROR(this->get_logger(), "Ignoring operational zone with a non-finite vertex.");
            publishStatus("Operational zone rejected: invalid vertex");
            return;
        }
        twice_area += a.x * b.y - b.x * a.y;
    }
    if (std::abs(twice_area) <= 1e-6){
        RCLCPP_ERROR(this->get_logger(), "Ignoring operational zone with a degenerate polygon.");
        publishStatus("Operational zone rejected: polygon area is zero");
        return;
    }
    if (msg->zone_type == ugv_nav4d_ros2::msg::ForbiddenZone::SPEED_LIMIT &&
        (!std::isfinite(msg->speed_limit) || msg->speed_limit <= 0.0f)){
        RCLCPP_ERROR(this->get_logger(), "Ignoring speed-limit zone with a non-positive or invalid limit.");
        publishStatus("Speed-limit zone rejected: limit must be greater than zero");
        return;
    }
    forbidden_zones.push_back(*msg);
    RCLCPP_INFO_STREAM(this->get_logger(), "Added operational zone " << forbidden_zones.size()
                       << " with " << msg->vertices.size() << " vertices.");
    onForbiddenZonesChanged(zoneAffectsPlanning(*msg), &forbidden_zones.back());
    publishStatus(std::to_string(forbidden_zones.size()) + " operational zone(s) active");
}

void PathPlannerNode::clearForbiddenZonesCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (forbidden_zones.empty()){
        // Nothing to clear; skip the (expensive) trav map regeneration.
        response->success = true;
        response->message = "No operational zones set; nothing to clear.";
        return;
    }
    const size_t count = forbidden_zones.size();
    const bool planning_graph_changed =
        std::any_of(forbidden_zones.begin(), forbidden_zones.end(), zoneAffectsPlanning);
    forbidden_zones.clear();
    onForbiddenZonesChanged(planning_graph_changed);
    response->success = true;
    response->message = "Cleared " + std::to_string(count) + " operational zone(s).";
    RCLCPP_INFO_STREAM(this->get_logger(), response->message);
    publishStatus(response->message);
}

void PathPlannerNode::removeLastForbiddenZoneCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (forbidden_zones.empty()){
        response->success = false;
        response->message = "No operational zones set.";
        return;
    }
    const bool planning_graph_changed = zoneAffectsPlanning(forbidden_zones.back());
    forbidden_zones.pop_back();
    onForbiddenZonesChanged(planning_graph_changed);
    response->success = true;
    response->message = std::to_string(forbidden_zones.size()) + " operational zone(s) remaining.";
    RCLCPP_INFO_STREAM(this->get_logger(), "Removed last operational zone; " << response->message);
    publishStatus("Removed last operational zone; " + response->message);
}

void PathPlannerNode::onForbiddenZonesChanged(bool planning_graph_changed,
                                              const ugv_nav4d_ros2::msg::ForbiddenZone* added_zone){
    publishForbiddenZoneMarkers();
    // MLS-edit (TRAVERSABLE) zones are destructive: removing one must restore
    // the original patches, which only rebuilding the MLS from its source can
    // do. Detect removals via the zone-key set. In live-mapping mode the
    // patches restore implicitly as new pointclouds are merged in.
    std::set<std::string> traversable_keys;
    for (const auto& zone : forbidden_zones){
        if (zone.zone_type == ugv_nav4d_ros2::msg::ForbiddenZone::TRAVERSABLE){
            traversable_keys.insert(traversableZoneKey(zone));
        }
    }
    bool traversable_zone_removed = false;
    for (const auto& key : last_traversable_zone_keys_){
        if (!traversable_keys.count(key)){
            traversable_zone_removed = true;
            break;
        }
    }
    last_traversable_zone_keys_ = std::move(traversable_keys);
    if (traversable_zone_removed && got_map){
        if (get_parameter("load_mls_from_file").as_bool()){
            RCLCPP_INFO(this->get_logger(), "Traversable (MLS edit) zone removed; reloading the MLS from file to restore the original patches.");
            initializeMLSMap();
            publishStatus("MLS restored to original from file; use 'Regenerate trav map' "
                          "to re-apply the remaining Traversable zones (if any).");
        } else {
            RCLCPP_WARN(this->get_logger(), "Traversable (MLS edit) zone removed; original patches only reappear as new map pointclouds are merged in.");
        }
    }
    if (!planning_graph_changed){
        rebuildSpeedZoneCache();
        publishZoneSpeedLimit(start_pose.pose.position);
        return;
    }
    // Fast path for ADDING a zone: zones are only painted onto the already-expanded
    // trav map (applyForbiddenZones runs after expandAll), so a new zone can be
    // applied in place -- no expandAll, no planner/environment rebuild. PREFERRED
    // zones are excluded: they interact with the map-wide baseline cost that
    // applyForbiddenZones computes over the full zone set.
    if (added_zone && is_configured && got_map && planner_ptr && traversability_generator_ptr &&
        added_zone->zone_type != ugv_nav4d_ros2::msg::ForbiddenZone::PREFERRED &&
        added_zone->zone_type != ugv_nav4d_ros2::msg::ForbiddenZone::TRAVERSABLE){
        std::vector<traversability_generator3d::TravGenNode*> keep_out_nodes;
        const size_t marked = applyZoneToTravMap(*added_zone, &keep_out_nodes);
        const size_t inflated = inflateZoneObstacles(keep_out_nodes);
        planner_ptr->updateMap(traversability_generator_ptr->getTraversabilityMap());
        rebuildSpeedZoneCache();
        RCLCPP_INFO_STREAM(this->get_logger(), "Applied new zone incrementally to " << marked
                           << " node(s) (+" << inflated
                           << " inflated margin cells); skipped the full map rebuild.");
        publishTravMap();
        validatePendingPath();
        return;
    }
    // Zones are baked into the trav map during generation, so a change requires a
    // rebuild. Clearing a zone has no cheaper path anyway (original node types are
    // not stored). Also refreshes the trav map in RViz.
    if (is_configured && got_map){
        configurePlanner();
        publishTravMap();
        validatePendingPath();
    } else if (has_pending_path){
        has_pending_path = false;
        path_approved = false;
        publishPreviewPending();
        pending_path_is_recovery = false;
        std_msgs::msg::Bool route_valid;
        route_valid.data = false;
        route_valid_publisher->publish(route_valid);
        clearExecutingPathDisplay();
    }
}

std::string PathPlannerNode::traversableZoneKey(const ugv_nav4d_ros2::msg::ForbiddenZone& zone){
    std::ostringstream key;
    key << std::fixed << std::setprecision(2);
    for (const auto& v : zone.vertices){
        key << v.x << ',' << v.y << ',' << v.z << ';';
    }
    return key.str();
}

ugv_nav4d_ros2::msg::ForbiddenZone* PathPlannerNode::lastTraversableZone(){
    const auto now = this->get_clock()->now();
    for (auto it = forbidden_zones.rbegin(); it != forbidden_zones.rend(); ++it){
        if (it->zone_type != ugv_nav4d_ros2::msg::ForbiddenZone::TRAVERSABLE){
            continue;
        }
        const bool expired = (it->expires_at.sec != 0 || it->expires_at.nanosec != 0) &&
                             rclcpp::Time(it->expires_at) <= now;
        if (!expired){
            return &*it;
        }
    }
    return nullptr;
}

void PathPlannerNode::applyMlsEditZones(){
    last_mls_patches_removed_ = 0;
    last_mls_patches_added_ = 0;
    if (!mls_map_ptr || !got_map || forbidden_zones.empty()){
        return;
    }
    const auto now = this->get_clock()->now();
    size_t zones_applied = 0;
    for (const auto& zone : forbidden_zones){
        if (zone.zone_type != ugv_nav4d_ros2::msg::ForbiddenZone::TRAVERSABLE){
            continue;
        }
        const bool expired = (zone.expires_at.sec != 0 || zone.expires_at.nanosec != 0) &&
                             rclcpp::Time(zone.expires_at) <= now;
        if (expired){
            continue;
        }
        applyMlsEditZone(zone);
        ++zones_applied;
    }
    if (zones_applied){
        RCLCPP_INFO_STREAM(this->get_logger(), "MLS edit: " << zones_applied
                           << " Traversable zone(s): removed " << last_mls_patches_removed_
                           << " patch(es), added " << last_mls_patches_added_
                           << " synthetic ground patch(es).");
    }
}

void PathPlannerNode::applyMlsEditZone(const ugv_nav4d_ros2::msg::ForbiddenZone& zone){
    if (zone.vertices.size() < 3){
        return;
    }
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = min_x, max_y = max_x;
    double cx = 0.0, cy = 0.0, ref_z = 0.0;
    for (const auto& v : zone.vertices){
        min_x = std::min(min_x, v.x); max_x = std::max(max_x, v.x);
        min_y = std::min(min_y, v.y); max_y = std::max(max_y, v.y);
        cx += v.x; cy += v.y; ref_z += v.z;
    }
    cx /= zone.vertices.size();
    cy /= zone.vertices.size();
    ref_z /= zone.vertices.size();

    const auto resolution = mls_map_ptr->getResolution();
    const maps::grid::Vector2ui num_cells = mls_map_ptr->getNumCells();
    typedef maps::grid::MLSMap<maps::grid::MLSConfig::SLOPE>::CellType Cell;

    // Pass 1: delete the original patches (grass) inside the polygon. Band
    // limited so structures on other storeys survive; the ceiling is operator
    // adjustable so overhead stuff beyond robot height stays in the map.
    // This also wipes a previous fill before a re-fill with new parameters.
    double vz_min = std::numeric_limits<double>::max();
    double vz_max = std::numeric_limits<double>::lowest();
    for (const auto& v : zone.vertices){
        vz_min = std::min(vz_min, v.z);
        vz_max = std::max(vz_max, v.z);
    }
    const double delete_top = zone.delete_top > 0.0f ? zone.delete_top : 2.0;
    const double band_low = vz_min - 2.0;
    const double band_high = vz_max + delete_top;
    const int ix0 = std::max(0, static_cast<int>(std::floor((min_x - mls_min_x) / resolution.x())));
    const int ix1 = std::min(static_cast<int>(num_cells.x()) - 1,
                             static_cast<int>(std::floor((max_x - mls_min_x) / resolution.x())));
    const int iy0 = std::max(0, static_cast<int>(std::floor((min_y - mls_min_y) / resolution.y())));
    const int iy1 = std::min(static_cast<int>(num_cells.y()) - 1,
                             static_cast<int>(std::floor((max_y - mls_min_y) / resolution.y())));
    for (int ix = ix0; ix <= ix1; ++ix){
        for (int iy = iy0; iy <= iy1; ++iy){
            const double wx = mls_min_x + (ix + 0.5) * resolution.x();
            const double wy = mls_min_y + (iy + 0.5) * resolution.y();
            if (!pointInPolygonXY(wx, wy, zone.vertices)){
                continue;
            }
            Cell& list = mls_map_ptr->at(static_cast<size_t>(ix), static_cast<size_t>(iy));
            for (Cell::iterator it = list.begin(); it != list.end(); ){
                float patch_min = 0.0f, patch_max = 0.0f;
                it->getRange(patch_min, patch_max);
                const double top = static_cast<double>(patch_max);
                if (top >= band_low && top <= band_high){
                    it = list.erase(it);
                    ++last_mls_patches_removed_;
                } else {
                    ++it;
                }
            }
        }
    }

    if (!zone.fill_enabled){
        return;
    }
    // Pass 2: refill with synthetic ground patches on the operator's plane.
    // Same construction as the proven initial-patch bootstrap in travgen:
    // oversample at half resolution, skip spots an existing patch covers.
    const Eigen::Vector3d normal =
        (Eigen::AngleAxisd(zone.fill_roll, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(zone.fill_pitch, Eigen::Vector3d::UnitY())) * Eigen::Vector3d::UnitZ();
    if (normal.z() < 0.5){
        RCLCPP_ERROR(this->get_logger(), "MLS edit: fill plane steeper than 60 degrees; skipping fill.");
        return;
    }
    const double z0 = ref_z + zone.fill_z_offset;
    const double step = resolution.x() / 2.0;
    for (double x = min_x; x <= max_x; x += step){
        for (double y = min_y; y <= max_y; y += step){
            if (!pointInPolygonXY(x, y, zone.vertices)){
                continue;
            }
            const double z = z0 - (normal.x() * (x - cx) + normal.y() * (y - cy)) / normal.z();
            const Eigen::Vector3d pos(x, y, z);
            maps::grid::Index idx;
            if (!mls_map_ptr->toGrid(pos, idx)){
                continue;
            }
            auto& list = mls_map_ptr->at(idx);
            bool covered = false;
            for (const auto& patch : list){
                if (patch.isCovered(static_cast<float>(z), 0.05f)){
                    covered = true;
                    break;
                }
            }
            if (covered){
                continue;
            }
            maps::grid::MLSMapSloped::PatchType patch(pos.cast<float>(), traversability_config.initialPatchVariance);
            list.insert(patch);
            ++last_mls_patches_added_;
        }
    }
}

static double squaredDistToPolygonEdgeXY(double x, double y,
                                         const std::vector<geometry_msgs::msg::Point>& poly){
    double best = std::numeric_limits<double>::max();
    const size_t n = poly.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++){
        const double ax = poly[j].x, ay = poly[j].y;
        const double dx = poly[i].x - ax, dy = poly[i].y - ay;
        const double len2 = dx * dx + dy * dy;
        double t = len2 > 0.0 ? ((x - ax) * dx + (y - ay) * dy) / len2 : 0.0;
        t = std::min(1.0, std::max(0.0, t));
        const double px = ax + t * dx - x;
        const double py = ay + t * dy - y;
        best = std::min(best, px * px + py * py);
    }
    return best;
}

bool PathPlannerNode::fitFillPlaneToSurroundingGround(
        const ugv_nav4d_ros2::msg::ForbiddenZone& zone,
        double& roll_rad, double& pitch_rad, double& z_offset_m, std::string& err)
{
    if (zone.vertices.size() < 3 || !mls_map_ptr){
        err = "no polygon / no map.";
        return false;
    }
    // The real ground the fill should continue lives just OUTSIDE the polygon;
    // inside there are only the grass tops being replaced. Sample the lowest
    // plausible patch top per cell in a ring around the boundary and fit a
    // plane to those.
    static constexpr double kRingM = 0.75;      // ring width outside the polygon
    static constexpr size_t kMinSamples = 12;
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = min_x, max_y = max_x;
    double cx = 0.0, cy = 0.0, ref_z = 0.0;
    double vz_min = min_x, vz_max = max_x;
    for (const auto& v : zone.vertices){
        min_x = std::min(min_x, v.x); max_x = std::max(max_x, v.x);
        min_y = std::min(min_y, v.y); max_y = std::max(max_y, v.y);
        vz_min = std::min(vz_min, v.z); vz_max = std::max(vz_max, v.z);
        cx += v.x; cy += v.y; ref_z += v.z;
    }
    cx /= zone.vertices.size();
    cy /= zone.vertices.size();
    ref_z /= zone.vertices.size();
    // Same level band the delete pass trusts: clicks land on grass tops, the
    // ground is at or below them; canopy and structures above are ignored.
    const double band_low = vz_min - 2.0;
    const double band_high = vz_max + 0.5;

    const auto resolution = mls_map_ptr->getResolution();
    const maps::grid::Vector2ui num_cells = mls_map_ptr->getNumCells();
    typedef maps::grid::MLSMap<maps::grid::MLSConfig::SLOPE>::CellType Cell;
    const int ix0 = std::max(0, static_cast<int>(std::floor((min_x - kRingM - mls_min_x) / resolution.x())));
    const int ix1 = std::min(static_cast<int>(num_cells.x()) - 1,
                             static_cast<int>(std::floor((max_x + kRingM - mls_min_x) / resolution.x())));
    const int iy0 = std::max(0, static_cast<int>(std::floor((min_y - kRingM - mls_min_y) / resolution.y())));
    const int iy1 = std::min(static_cast<int>(num_cells.y()) - 1,
                             static_cast<int>(std::floor((max_y + kRingM - mls_min_y) / resolution.y())));
    std::vector<Eigen::Vector3d> samples;   // (x - cx, y - cy, ground z)
    for (int ix = ix0; ix <= ix1; ++ix){
        for (int iy = iy0; iy <= iy1; ++iy){
            const double wx = mls_min_x + (ix + 0.5) * resolution.x();
            const double wy = mls_min_y + (iy + 0.5) * resolution.y();
            if (pointInPolygonXY(wx, wy, zone.vertices) ||
                squaredDistToPolygonEdgeXY(wx, wy, zone.vertices) > kRingM * kRingM){
                continue;
            }
            const Cell& list = mls_map_ptr->at(static_cast<size_t>(ix), static_cast<size_t>(iy));
            double ground = std::numeric_limits<double>::max();
            for (const auto& patch : list){
                float patch_min = 0.0f, patch_max = 0.0f;
                patch.getRange(patch_min, patch_max);
                const double top = static_cast<double>(patch_max);
                if (top >= band_low && top <= band_high){
                    ground = std::min(ground, top);
                }
            }
            if (ground < std::numeric_limits<double>::max()){
                samples.emplace_back(wx - cx, wy - cy, ground);
            }
        }
    }
    if (samples.size() < kMinSamples){
        err = "only " + std::to_string(samples.size()) +
              " ground cell(s) in the ring outside the polygon (need " +
              std::to_string(kMinSamples) + ").";
        return false;
    }
    // Least squares z = a*dx + b*dy + c around the centroid, with one
    // outlier-trim pass so stray grass/rocks in the ring do not tilt the fit.
    auto solve = [](const std::vector<Eigen::Vector3d>& pts, Eigen::Vector3d& sol) -> bool {
        Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
        Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
        for (const auto& p : pts){
            const Eigen::Vector3d row(p.x(), p.y(), 1.0);
            A += row * row.transpose();
            rhs += row * p.z();
        }
        const Eigen::FullPivLU<Eigen::Matrix3d> lu(A);
        if (!lu.isInvertible()){
            return false;
        }
        sol = lu.solve(rhs);
        return sol.allFinite();
    };
    Eigen::Vector3d sol;
    if (!solve(samples, sol)){
        err = "degenerate ring geometry (ground samples lie on a line).";
        return false;
    }
    double ss = 0.0;
    for (const auto& p : samples){
        const double r = p.z() - (sol.x() * p.x() + sol.y() * p.y() + sol.z());
        ss += r * r;
    }
    const double gate = std::max(2.0 * std::sqrt(ss / samples.size()), 0.03);
    std::vector<Eigen::Vector3d> kept;
    kept.reserve(samples.size());
    for (const auto& p : samples){
        if (std::abs(p.z() - (sol.x() * p.x() + sol.y() * p.y() + sol.z())) <= gate){
            kept.push_back(p);
        }
    }
    if (kept.size() >= kMinSamples && kept.size() < samples.size()){
        Eigen::Vector3d refit;
        if (solve(kept, refit)){
            sol = refit;
        }
    }
    const Eigen::Vector3d normal = Eigen::Vector3d(-sol.x(), -sol.y(), 1.0).normalized();
    if (normal.z() < std::cos(45.0 * M_PI / 180.0)){
        err = "fitted ground is steeper than 45 degrees; set the plane manually.";
        return false;
    }
    // Invert the fill construction normal = Rx(roll)*Ry(pitch)*ez
    //   = (sin(pitch), -sin(roll)cos(pitch), cos(roll)cos(pitch)).
    pitch_rad = std::asin(std::min(1.0, std::max(-1.0, normal.x())));
    roll_rad = std::atan2(-normal.y(), normal.z());
    z_offset_m = sol.z() - ref_z;   // plane z at the centroid vs. mean vertex z
    RCLCPP_INFO(this->get_logger(),
                "Fill plane fit: %zu/%zu ring cells, z %.2f m below clicked vertices, "
                "roll %.1f deg, pitch %.1f deg.",
                kept.size(), samples.size(), -z_offset_m,
                roll_rad * 180.0 / M_PI, pitch_rad * 180.0 / M_PI);
    return true;
}

void PathPlannerNode::regenerateTravMapCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (is_planning){
        response->success = false;
        response->message = "Cannot regenerate the traversability map while planning is active.";
        publishStatus(response->message);
        return;
    }
    if (!got_map || !mls_map_ptr || !is_configured || !planner_ptr){
        response->success = false;
        response->message = "No map loaded / planner not configured; nothing to regenerate.";
        publishStatus(response->message);
        return;
    }
    if (!get_parameter("read_pose_from_topic").as_bool() && !updatePoseFromTF()){
        response->success = false;
        response->message = "Cannot regenerate: TF lookup of the robot pose failed.";
        publishStatus(response->message);
        return;
    }
    ScopedRebuildFlag rebuild_flag(*this);
    publishStatus("Regenerating traversability map from the current (edited) MLS...");
    // Fresh generator: expansion must re-evaluate cells whose MLS patches were
    // deleted or filled; reusing the old one keeps stale nodes alive. The MLS
    // itself is kept as-is -- that is the whole point of this service (the
    // regenerate_maps service is the one that rebuilds the MLS from source).
    traversability_generator_ptr.reset(
        new traversability_generator3d::TraversabilityGenerator3d(traversability_config));
    traversability_generator_ptr->setMLSGrid(mls_map_ptr);
    applyMlsEditZones();  // idempotent; covers zones loaded from a mission file

    Eigen::Affine3d body2MLS;
    body2MLS.translation() << start_pose.pose.position.x, start_pose.pose.position.y,
                              start_pose.pose.position.z;
    Eigen::Quaterniond quat(start_pose.pose.orientation.w, start_pose.pose.orientation.x,
                            start_pose.pose.orientation.y, start_pose.pose.orientation.z);
    body2MLS.linear() = quat.toRotationMatrix();
    Eigen::Affine3d body2Ground(Eigen::Affine3d::Identity());
    body2Ground.translation() = Eigen::Vector3d(0, 0, -get_parameter("distToGround").as_double());
    const auto t_expand = std::chrono::steady_clock::now();
    traversability_generator_ptr->expandAll((body2MLS * body2Ground).translation());
    RCLCPP_INFO_STREAM(this->get_logger(), "Timing: trav-map re-expansion took "
        << std::chrono::duration<double>(std::chrono::steady_clock::now() - t_expand).count()
        << " s.");
    applyForbiddenZones();
    rebuildSpeedZoneCache();
    planner_ptr->updateMap(traversability_generator_ptr->getTraversabilityMap());
    publishTravMap();
    validatePendingPath();
    response->success = true;
    response->message = "Traversability map regenerated from the current MLS.";
    publishStatus(response->message);
}

void PathPlannerNode::groomUnderRobotTick(){
    // The robot's presence proves the ground under it is drivable: while
    // enabled, replace the MLS content under the WHEEL footprint (no tool --
    // the shovel hovers, it proves nothing) with a plane derived from the
    // measured ground level and the chassis attitude. Pure MLS manipulation:
    // the trav map picks it up on the next regeneration.
    static constexpr double kStepM = 0.5;          // re-groom every this much travel
    static constexpr double kYawStepRad = 0.26;    // ... or ~15 deg of heading change
    static constexpr double kBandBelowM = 0.3;     // delete down to ground minus this
    if (!groom_under_robot_ || !got_map || !mls_map_ptr){
        return;
    }
    const auto& pos = start_pose.pose.position;
    const Eigen::Quaterniond q(start_pose.pose.orientation.w, start_pose.pose.orientation.x,
                               start_pose.pose.orientation.y, start_pose.pose.orientation.z);
    const double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                                  1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    if (std::isfinite(last_groom_x_)){
        const double moved = std::hypot(pos.x - last_groom_x_, pos.y - last_groom_y_);
        const double turned = std::abs(std::remainder(yaw - last_groom_yaw_, 2.0 * M_PI));
        if (moved < kStepM && turned < kYawStepRad){
            return;
        }
    }
    // 1) Wheel envelope (no tool frames) in the robot frame. Also carries the
    //    live mean wheel-center z, which tracks ride-height changes from the
    //    active ground adaptation.
    FootprintEnvelope envelope;
    std::string err;
    if (!measureFootprintEnvelope(envelope, &err)){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Grooming skipped: %s", err.c_str());
        return;
    }
    // 2) Ground level: base_link z (localization truth) minus the CALIBRATED
    //    distToGround parameter, corrected by how far the ride height moved
    //    since that calibration (ground adaptation raises/lowers base_link
    //    relative to the wheels, which a fixed distToGround cannot see; the
    //    wheel-z baseline is captured when distToGround is known good).
    //    Deliberately NOT the live nearest-patch query: after the first groom
    //    step that query lands on our own synthetic fill, whose patch top
    //    sits slightly above its seed point, and the anchor ratchets upward a
    //    few cm per step until the fill is airborne (and the delete band no
    //    longer reaches the real terrain).
    if (!std::isfinite(groom_wheel_z_baseline_)){
        groom_wheel_z_baseline_ = envelope.wheel_mean_z;
        RCLCPP_INFO(this->get_logger(),
            "Grooming: captured wheel-z baseline %.3f m; assumes distToGround matches the "
            "CURRENT ride height (run Recalibrate height if unsure).",
            groom_wheel_z_baseline_);
    }
    // > 0 when the chassis sits higher above the wheels than at calibration.
    const double ride_height_delta = groom_wheel_z_baseline_ - envelope.wheel_mean_z;
    const double ground_z = pos.z - (get_parameter("distToGround").as_double() + ride_height_delta);
    // Sanity check only: warn when the calibration disagrees with the map.
    {
        double patch_z = 0.0;
        if (groundPatchHeightUnderRobot(patch_z, &err) &&
            std::abs(patch_z - ground_z) > 0.5){
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                "Grooming: calibrated ground (%.2f) is %.2f m off the nearest MLS patch (%.2f); "
                "consider Recalibrate height.", ground_z, patch_z - ground_z, patch_z);
        }
    }
    // Wheels-only envelope plus the grooming's OWN uniform margin -- kept
    // independent of the planner's footprint_margin_* safety margins so the
    // groomed strip width is tunable without touching collision checking.
    const double margin = get_parameter("groom_margin").as_double();
    const double fp_min_x = envelope.wheel_min_x - margin;
    const double fp_max_x = envelope.wheel_max_x + margin;
    const double fp_half_y = envelope.wheel_max_abs_y + margin;
    last_groom_x_ = pos.x;
    last_groom_y_ = pos.y;
    last_groom_yaw_ = yaw;
    // 3) Fill plane through the measured ground, tilted like the chassis.
    const Eigen::Vector3d normal = q * Eigen::Vector3d::UnitZ();
    if (normal.z() < 0.5){
        return;  // implausible attitude sample; skip this tick
    }
    const double band_above = get_parameter("groom_delete_top").as_double();
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const auto resolution = mls_map_ptr->getResolution();
    const maps::grid::Vector2ui num_cells = mls_map_ptr->getNumCells();
    typedef maps::grid::MLSMap<maps::grid::MLSConfig::SLOPE>::CellType Cell;

    // Delete pass over the world-aligned bbox of the oriented rectangle.
    const double reach = std::max({std::abs(fp_min_x), std::abs(fp_max_x), fp_half_y}) * M_SQRT2;
    const int ix0 = std::max(0, static_cast<int>(std::floor((pos.x - reach - mls_min_x) / resolution.x())));
    const int ix1 = std::min(static_cast<int>(num_cells.x()) - 1,
                             static_cast<int>(std::floor((pos.x + reach - mls_min_x) / resolution.x())));
    const int iy0 = std::max(0, static_cast<int>(std::floor((pos.y - reach - mls_min_y) / resolution.y())));
    const int iy1 = std::min(static_cast<int>(num_cells.y()) - 1,
                             static_cast<int>(std::floor((pos.y + reach - mls_min_y) / resolution.y())));
    size_t removed = 0;
    size_t added = 0;
    for (int ix = ix0; ix <= ix1; ++ix){
        for (int iy = iy0; iy <= iy1; ++iy){
            const double wx = mls_min_x + (ix + 0.5) * resolution.x();
            const double wy = mls_min_y + (iy + 0.5) * resolution.y();
            const double dx = wx - pos.x;
            const double dy = wy - pos.y;
            const double bx = cos_yaw * dx + sin_yaw * dy;
            const double by = -sin_yaw * dx + cos_yaw * dy;
            if (bx < fp_min_x || bx > fp_max_x || std::abs(by) > fp_half_y){
                continue;
            }
            Cell& list = mls_map_ptr->at(static_cast<size_t>(ix), static_cast<size_t>(iy));
            for (Cell::iterator it = list.begin(); it != list.end(); ){
                float patch_min = 0.0f, patch_max = 0.0f;
                it->getRange(patch_min, patch_max);
                const double top = static_cast<double>(patch_max);
                if (top >= ground_z - kBandBelowM && top <= ground_z + band_above){
                    it = list.erase(it);
                    ++removed;
                } else {
                    ++it;
                }
            }
        }
    }
    // Fill pass: oversample the rectangle at half resolution (initial-patch
    // construction, merged where a patch already covers the spot).
    const double step = resolution.x() / 2.0;
    for (double bx = fp_min_x; bx <= fp_max_x; bx += step){
        for (double by = -fp_half_y; by <= fp_half_y; by += step){
            const double wx = pos.x + cos_yaw * bx - sin_yaw * by;
            const double wy = pos.y + sin_yaw * bx + cos_yaw * by;
            const double z = ground_z - (normal.x() * (wx - pos.x) + normal.y() * (wy - pos.y)) / normal.z();
            const Eigen::Vector3d point(wx, wy, z);
            maps::grid::Index idx;
            if (!mls_map_ptr->toGrid(point, idx)){
                continue;
            }
            auto& list = mls_map_ptr->at(idx);
            bool covered = false;
            for (const auto& patch : list){
                if (patch.isCovered(static_cast<float>(z), 0.05f)){
                    covered = true;
                    break;
                }
            }
            if (covered){
                continue;
            }
            maps::grid::MLSMapSloped::PatchType patch(point.cast<float>(), traversability_config.initialPatchVariance);
            list.insert(patch);
            ++added;
        }
    }
    groom_removed_total_ += removed;
    groom_added_total_ += added;
    if (removed || added){
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Grooming at (%.1f, %.1f): removed %zu, added %zu patch(es) "
                             "(session totals: %zu / %zu).",
                             pos.x, pos.y, removed, added,
                             groom_removed_total_, groom_added_total_);
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

    // Preferred polygons become genuinely attractive by adding a modest cost
    // outside all preferred corridors. Merely reducing the usual zero-valued
    // terrain cost inside a corridor would have no planning effect.
    std::vector<const ugv_nav4d_ros2::msg::ForbiddenZone*> preferred_zones;
    for (const auto& zone : forbidden_zones){
        const bool expired = (zone.expires_at.sec != 0 || zone.expires_at.nanosec != 0) &&
                             rclcpp::Time(zone.expires_at) <= this->get_clock()->now();
        if (!expired && zone.zone_type == ugv_nav4d_ros2::msg::ForbiddenZone::PREFERRED){
            preferred_zones.push_back(&zone);
        }
    }
    if (!preferred_zones.empty()){
        for (const auto& levels : trav_map_3d){
            for (auto* node : levels){
                Eigen::Vector3d position;
                if (!trav_map_3d.fromGrid(node->getIndex(), position, node->getHeight(), false)) continue;
                bool preferred = false;
                for (const auto* zone : preferred_zones){
                    if (pointInPolygonXY(position.x(), position.y(), zone->vertices) &&
                        zoneLevelBandContains(zone->vertices, position.z())){
                        preferred = true;
                        break;
                    }
                }
                if (!preferred && node->getUserData().nodeType != traversability_generator3d::NodeType::OBSTACLE){
                    node->getUserData().cost += 500;
                }
            }
        }
    }

    std::vector<traversability_generator3d::TravGenNode*> keep_out_nodes;
    for (const auto& zone : forbidden_zones)
    {
        if ((zone.expires_at.sec != 0 || zone.expires_at.nanosec != 0) &&
            rclcpp::Time(zone.expires_at) <= this->get_clock()->now()){
            continue;
        }
        if (zone.zone_type == ugv_nav4d_ros2::msg::ForbiddenZone::ANNOTATION ||
            zone.zone_type == ugv_nav4d_ros2::msg::ForbiddenZone::SPEED_LIMIT ||
            zone.zone_type == ugv_nav4d_ros2::msg::ForbiddenZone::TRAVERSABLE){
            // These are execution/operator semantics and intentionally do not
            // change graph connectivity.
            continue;
        }
        marked += applyZoneToTravMap(zone, &keep_out_nodes);
    }

    const size_t inflated = inflateZoneObstacles(keep_out_nodes);
    RCLCPP_INFO_STREAM(this->get_logger(), "Applied " << forbidden_zones.size()
                       << " operational zone(s) to " << marked << " traversability node(s) (+"
                       << inflated << " inflated margin cells).");
}

size_t PathPlannerNode::applyZoneToTravMap(const ugv_nav4d_ros2::msg::ForbiddenZone& zone,
        std::vector<traversability_generator3d::TravGenNode*>* keep_out_nodes){
    const auto& trav_map_3d = traversability_generator_ptr->getTraversabilityMap();

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
        return 0;
    }

    size_t marked = 0;
    while (!queue.empty())
    {
        traversability_generator3d::TravGenNode* node = queue.front();
        queue.pop_front();

        auto& data = node->getUserData();
        if (zone.zone_type == ugv_nav4d_ros2::msg::ForbiddenZone::KEEP_OUT){
            // Same marking travgen uses for real obstacles, so all planner
            // checks (goal validity, expansions, obstacle checks) respect it.
            node->setType(maps::grid::TraversabilityNodeBase::OBSTACLE);
            data.nodeType = traversability_generator3d::NodeType::OBSTACLE;
            if (keep_out_nodes){
                keep_out_nodes->push_back(node);
            }
        } else if (zone.zone_type == ugv_nav4d_ros2::msg::ForbiddenZone::CAUTION){
            const double multiplier = std::max(1.0f, zone.cost_multiplier);
            data.cost += static_cast<int>(1000.0 * multiplier);
        } else if (zone.zone_type == ugv_nav4d_ros2::msg::ForbiddenZone::PREFERRED){
            const double divisor = std::max(1.0f, zone.cost_multiplier);
            data.cost = static_cast<int>(data.cost / divisor);
        } else if (zone.zone_type == ugv_nav4d_ros2::msg::ForbiddenZone::DIRECTION_RESTRICTED){
            data.allowedOrientations.clear();
            constexpr double width = M_PI / 3.0; // +/- 30 degrees
            data.allowedOrientations.emplace_back(
                base::Angle::fromRad(zone.preferred_heading - width / 2.0), width);
            data.nodeType = traversability_generator3d::NodeType::PARTIALLY_TRAVERSABLE;
        }
        ++marked;

        for (maps::grid::TraversabilityNodeBase* nb : node->getConnections())
        {
            auto* neighbor = static_cast<traversability_generator3d::TravGenNode*>(nb);
            if (visited.count(neighbor)){
                continue;
            }
            Eigen::Vector3d position;
            trav_map_3d.fromGrid(neighbor->getIndex(), position, neighbor->getHeight(), false);
            if (!pointInPolygonXY(position.x(), position.y(), zone.vertices) ||
                !zoneLevelBandContains(zone.vertices, position.z())){
                continue;
            }
            visited.insert(neighbor);
            queue.push_back(neighbor);
        }
    }
    return marked;
}

size_t PathPlannerNode::inflateZoneObstacles(
        const std::vector<traversability_generator3d::TravGenNode*>& seeds){
    if (seeds.empty()){
        return 0;
    }
    // Mirror the inflation radius of TraversabilityGenerator3d::inflateObstacles():
    // the gap between the AABB footprint boundary and the rotation-safe circle,
    // scaled by obstacleInflationMultiplier (>= 1.0), at least one grid cell.
    // Offset-aware like travgen: an off-center footprint extends the swept
    // circle and shrinks the guaranteed origin-centered core.
    const double half_x = traversability_config.robotSizeX / 2.0;
    const double half_y = traversability_config.robotSizeY / 2.0;
    const double abs_off = std::abs(traversability_config.footprintOffsetX);
    const double half_diagonal = std::hypot(half_x + abs_off, half_y);
    const double min_half_extent = std::max(0.0, std::min(half_x - abs_off, half_y));
    const double infl_gap = half_diagonal - min_half_extent / 2.0;
    const double multiplier = std::max(1.0, traversability_config.obstacleInflationMultiplier);
    const double infl_radius = multiplier *
        std::max(infl_gap, traversability_config.gridResolution * 1.1) + 1e-5;

    // Unlike travgen's inflateObstacles(), the margin cells become hard obstacles
    // instead of orientation-restricted (partially traversable) rims: zone cells
    // sit on plain terrain, so "safe orientations" would only re-derive the
    // distance to the zone itself. An operator keep-out margin must be absolute.
    // INFLATED_OBSTACLE keeps the ring distinguishable from the drawn zone.
    size_t inflated = 0;
    std::unordered_set<traversability_generator3d::TravGenNode*> evaluated;
    for (traversability_generator3d::TravGenNode* seed : seeds){
        const maps::grid::Index seed_idx = seed->getIndex();
        seed->eachConnectedNode(
            [&](maps::grid::TraversabilityNodeBase* nb, bool& expandNode, bool& /*stop*/)
        {
            auto* node = static_cast<traversability_generator3d::TravGenNode*>(nb);
            // 2D (XY) cell distance, matching travgen: inflation is a horizontal
            // clearance concept, independent of patch height.
            const maps::grid::Index diff = node->getIndex() - seed_idx;
            const double dist_2d =
                diff.matrix().cast<double>().norm() * traversability_config.gridResolution;
            if (dist_2d >= infl_radius){
                return;
            }
            expandNode = true;
            if (node->getType() == maps::grid::TraversabilityNodeBase::OBSTACLE){
                return; // zone interior or a real obstacle; nothing to add
            }
            if (evaluated.insert(node).second){
                node->setType(maps::grid::TraversabilityNodeBase::OBSTACLE);
                node->getUserData().nodeType =
                    traversability_generator3d::NodeType::INFLATED_OBSTACLE;
                ++inflated;
            }
        });
    }
    return inflated;
}

void PathPlannerNode::deleteForbiddenZoneCallback(const std::shared_ptr<ugv_nav4d_ros2::srv::DeleteForbiddenZone::Request> request,
                                                  std::shared_ptr<ugv_nav4d_ros2::srv::DeleteForbiddenZone::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (request->index == 0 || request->index > forbidden_zones.size()){
        response->success = false;
        response->message = "Operational zone " + std::to_string(request->index) + " does not exist ("
                            + std::to_string(forbidden_zones.size()) + " active).";
        publishStatus(response->message);
        return;
    }
    const auto zone_it = forbidden_zones.begin() + (request->index - 1);
    const bool planning_graph_changed = zoneAffectsPlanning(*zone_it);
    forbidden_zones.erase(zone_it);
    onForbiddenZonesChanged(planning_graph_changed);
    response->success = true;
    response->message = "Deleted operational zone " + std::to_string(request->index) + "; "
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

bool PathPlannerNode::measureFootprintEnvelope(FootprintEnvelope& envelope, std::string* error){
    const std::string robot_frame = get_parameter("robot_frame").as_string();
    const auto wheel_frames = get_parameter("footprint_wheel_frames").as_string_array();
    if (wheel_frames.empty()){
        if (error){
            *error = "footprint_wheel_frames is empty";
        }
        return false;
    }
    const auto lookup = [&](const std::string& frame, double& x, double& y, double* z = nullptr) -> bool {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_ptr->lookupTransform(robot_frame, frame, tf2::TimePointZero);
        } catch (const tf2::TransformException& ex){
            if (error){
                *error = "TF " + robot_frame + " <- " + frame + ": " + ex.what();
            }
            return false;
        }
        x = tf.transform.translation.x;
        y = tf.transform.translation.y;
        if (z){
            *z = tf.transform.translation.z;
        }
        return true;
    };

    envelope.wheel_min_x = std::numeric_limits<double>::max();
    envelope.wheel_max_x = std::numeric_limits<double>::lowest();
    envelope.max_abs_y = 0.0;
    double wheel_z_sum = 0.0;
    for (const auto& wheel_frame : wheel_frames){
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (!lookup(wheel_frame, x, y, &z)){
            return false;
        }
        envelope.wheel_min_x = std::min(envelope.wheel_min_x, x);
        envelope.wheel_max_x = std::max(envelope.wheel_max_x, x);
        envelope.wheel_max_abs_y = std::max(envelope.wheel_max_abs_y, std::abs(y));
        wheel_z_sum += z;
    }
    envelope.wheel_mean_z = wheel_z_sum / static_cast<double>(wheel_frames.size());
    envelope.min_x = envelope.wheel_min_x;
    envelope.max_x = envelope.wheel_max_x;
    envelope.max_abs_y = envelope.wheel_max_abs_y;
    // Copy: as_string_array() returns a reference into the TEMPORARY returned
    // by get_parameter(); iterating it directly is use-after-free.
    const auto extra_frames = get_parameter("footprint_extra_frames").as_string_array();
    for (const auto& extra_frame : extra_frames){
        double x = 0.0;
        double y = 0.0;
        if (!lookup(extra_frame, x, y)){
            return false;
        }
        envelope.min_x = std::min(envelope.min_x, x);
        envelope.max_x = std::max(envelope.max_x, x);
        envelope.max_abs_y = std::max(envelope.max_abs_y, std::abs(y));
    }
    return true;
}

void PathPlannerNode::updateFootprintCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                              std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    FootprintEnvelope envelope;
    std::string error;
    if (!measureFootprintEnvelope(envelope, &error)){
        response->success = false;
        response->message = "Footprint update failed: " + error;
        publishStatus(response->message);
        return;
    }
    // The footprint box supports an x offset (footprintOffsetX), so the
    // measured asymmetric envelope maps exactly: per-side margins extend the
    // envelope, the box center is the midpoint of the result. Y stays centered.
    const double front_x = envelope.max_x + get_parameter("footprint_margin_front").as_double();
    const double rear_x = envelope.min_x - get_parameter("footprint_margin_rear").as_double();
    const double new_size_x = front_x - rear_x;
    const double new_offset_x = (front_x + rear_x) / 2.0;
    const double new_size_y = 2.0 * (envelope.max_abs_y + get_parameter("footprint_margin_y").as_double());
    const double old_size_x = get_parameter("robotSizeX").as_double();
    const double old_size_y = get_parameter("robotSizeY").as_double();
    const double old_offset_x = get_parameter("footprintOffsetX").as_double();

    std::ostringstream summary;
    summary.setf(std::ios::fixed);
    summary.precision(2);
    summary << "footprint " << old_size_x << " x " << old_size_y
            << " m @ x" << (old_offset_x >= 0 ? "+" : "") << old_offset_x
            << " -> " << new_size_x << " x " << new_size_y << " m @ x"
            << (new_offset_x >= 0 ? "+" : "") << new_offset_x
            << " (envelope incl. tool: x [" << envelope.min_x << ", " << envelope.max_x
            << "], |y| " << envelope.max_abs_y << " m)";

    if (std::abs(new_size_x - old_size_x) < 0.01 &&
        std::abs(new_size_y - old_size_y) < 0.01 &&
        std::abs(new_offset_x - old_offset_x) < 0.01){
        response->success = true;
        response->message = "Footprint unchanged: " + summary.str() + "; no rebuild needed.";
        publishStatus(response->message);
        return;
    }

    // parametersCallback marks these dirty; parameterUpdateTimerCallback then
    // reconfigures the planner (trav-map regeneration + environment rebuild).
    set_parameters({rclcpp::Parameter("robotSizeX", new_size_x),
                    rclcpp::Parameter("robotSizeY", new_size_y),
                    rclcpp::Parameter("footprintOffsetX", new_offset_x)});
    response->success = true;
    response->message = "Footprint updated: " + summary.str() + "; planner reconfigures with the next tick.";
    RCLCPP_INFO_STREAM(this->get_logger(), response->message);
    publishStatus(response->message);
}

bool PathPlannerNode::groundPatchHeightUnderRobot(double& patch_z, std::string* error){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (!mls_map_ptr || !got_map){
        if (error) *error = "no MLS map loaded";
        return false;
    }
    // Query the MLS directly (source of truth for surface heights, present
    // wherever the point cloud has data) instead of the traversability map,
    // which only has nodes where expansion ran. Per operator request: take
    // the level list at the robot's (x, y) and use the nearest patch below
    // the robot's z.
    const auto& pos = start_pose.pose.position;
    const auto resolution = mls_map_ptr->getResolution();
    const maps::grid::Vector2ui num_cells = mls_map_ptr->getNumCells();
    const double lx = (pos.x - mls_min_x) / resolution.x();
    const double ly = (pos.y - mls_min_y) / resolution.y();
    const int cx = static_cast<int>(std::floor(lx));
    const int cy = static_cast<int>(std::floor(ly));
    constexpr int kMaxRingRadius = 2;         // robot shadows its own cell

    typedef maps::grid::MLSMap<maps::grid::MLSConfig::SLOPE>::CellType Cell;
    bool cell_in_map = false;
    bool saw_patch = false;
    for (int radius = 0; radius <= kMaxRingRadius; ++radius){
        double best_top = -std::numeric_limits<double>::infinity();
        for (int dx = -radius; dx <= radius; ++dx){
            for (int dy = -radius; dy <= radius; ++dy){
                if (std::max(std::abs(dx), std::abs(dy)) != radius){
                    continue; // ring cells only; inner cells were previous rings
                }
                const int nx = cx + dx;
                const int ny = cy + dy;
                if (nx < 0 || ny < 0 ||
                    static_cast<size_t>(nx) >= num_cells.x() ||
                    static_cast<size_t>(ny) >= num_cells.y()){
                    continue;
                }
                cell_in_map = true;
                const Cell& list = mls_map_ptr->at(static_cast<size_t>(nx),
                                                   static_cast<size_t>(ny));
                for (Cell::const_iterator it = list.begin(); it != list.end(); ++it){
                    saw_patch = true;
                    float min_z = 0.0f;
                    float max_z = 0.0f;
                    it->getRange(min_z, max_z);
                    const double top = static_cast<double>(max_z);
                    if (top <= pos.z && top > best_top){
                        best_top = top;
                    }
                }
            }
        }
        if (std::isfinite(best_top)){
            patch_z = best_top;
            return true;
        }
    }
    if (error){
        if (!cell_in_map){
            *error = "robot outside the MLS map";
        } else if (saw_patch){
            *error = "all MLS patches near the robot lie above the robot z "
                     "(pose frame or height grossly wrong)";
        } else {
            *error = "no MLS patches within " + std::to_string(kMaxRingRadius) +
                     " cells of the robot";
        }
    }
    return false;
}

void PathPlannerNode::recalibrateHeightCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                                std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    double patch_z = 0.0;
    std::string error;
    if (!groundPatchHeightUnderRobot(patch_z, &error)){
        response->success = false;
        response->message = "Height recalibration failed: " + error;
        publishStatus(response->message);
        return;
    }
    const double old_value = get_parameter("distToGround").as_double();
    const double new_value = start_pose.pose.position.z - patch_z;
    // Either way the calibration is confirmed valid for the CURRENT ride
    // height: refresh the grooming wheel-z baseline so its ground-adaptation
    // correction is measured from this moment. On TF failure fall back to
    // NaN; the groom tick then re-captures the baseline lazily.
    {
        FootprintEnvelope envelope;
        std::string envelope_error;
        groom_wheel_z_baseline_ = measureFootprintEnvelope(envelope, &envelope_error)
            ? envelope.wheel_mean_z
            : std::numeric_limits<double>::quiet_NaN();
    }
    if (std::abs(new_value - old_value) < 0.01){
        response->success = true;
        response->message = "distToGround already calibrated (change < 1 cm); nothing to do.";
        publishStatus(response->message);
        return;
    }
    std::ostringstream msg;
    msg.setf(std::ios::fixed);
    msg.precision(3);
    msg << "distToGround recalibrated: " << old_value << " -> " << new_value
        << " m (patch z " << patch_z << "); full map+planner rebuild follows, "
        << "do NOT press Regenerate maps -- it would run a second rebuild.";
    // No "footprint_" prefix: the parameter feeds the traversability config,
    // so setting it marks the planner dirty and triggers the rebuild.
    set_parameters({rclcpp::Parameter("distToGround", new_value)});
    response->success = true;
    response->message = msg.str();
    RCLCPP_INFO_STREAM(this->get_logger(), response->message);
    publishStatus(response->message);
}

void PathPlannerNode::publishWheelbaseStatus(){
    FootprintEnvelope envelope;
    std::string error;
    if (!measureFootprintEnvelope(envelope, &error)){
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
            "Wheelbase status unavailable: " << error);
        return;
    }
    const double wheelbase = envelope.wheel_max_x - envelope.wheel_min_x;

    std_msgs::msg::Float32 wheelbase_msg;
    wheelbase_msg.data = static_cast<float>(wheelbase);
    wheelbase_publisher->publish(wheelbase_msg);

    const std::string robot_frame = get_parameter("robot_frame").as_string();
    const double half_x = get_parameter("robotSizeX").as_double() / 2.0;
    const double half_y = get_parameter("robotSizeY").as_double() / 2.0;
    const double box_offset_x = get_parameter("footprintOffsetX").as_double();
    // Zero stamp = "latest available transform" in RViz. Stamping with now()
    // raced the localization TF (a few ms older, especially with sim time)
    // and produced "extrapolation into the future" errors on every publish.
    const rclcpp::Time stamp(0, 0, this->get_clock()->get_clock_type());
    // Persistent markers (lifetime 0): an expiry tied to the publish period
    // made the box flicker whenever a long callback (replanning, map rebuild)
    // blocked the executor past the lifetime, while the polygon display kept
    // its last message. Each publish overwrites the previous by ns/id anyway.
    const rclcpp::Duration lifetime = rclcpp::Duration::from_seconds(0.0);

    // Same convention as Nav2's published_footprint: the TRUE asymmetric
    // envelope (wheels + tool + per-side margins), matching what the
    // update_footprint service would apply.
    const double margin_y = get_parameter("footprint_margin_y").as_double();
    const double front_x = envelope.max_x + get_parameter("footprint_margin_front").as_double();
    const double rear_x = envelope.min_x - get_parameter("footprint_margin_rear").as_double();
    const double side_y = envelope.max_abs_y + margin_y;
    geometry_msgs::msg::PolygonStamped polygon;
    polygon.header.frame_id = robot_frame;
    polygon.header.stamp = stamp;
    const double polygon_corners[4][2] = {
        {front_x, side_y}, {front_x, -side_y}, {rear_x, -side_y}, {rear_x, side_y}};
    for (const auto& corner : polygon_corners){
        geometry_msgs::msg::Point32 p;
        p.x = static_cast<float>(corner[0]);
        p.y = static_cast<float>(corner[1]);
        polygon.polygon.points.push_back(p);
    }
    footprint_polygon_publisher->publish(polygon);

    // Groom footprint: wheels-only envelope + the same margin parameters --
    // exactly the rectangle groomUnderRobotTick() edits. Published empty when
    // grooming is off so the rviz Polygon display clears.
    geometry_msgs::msg::PolygonStamped groom_polygon;
    groom_polygon.header.frame_id = robot_frame;
    groom_polygon.header.stamp = stamp;
    if (groom_under_robot_){
        const double groom_margin = get_parameter("groom_margin").as_double();
        const double groom_front = envelope.wheel_max_x + groom_margin;
        const double groom_rear = envelope.wheel_min_x - groom_margin;
        const double groom_side = envelope.wheel_max_abs_y + groom_margin;
        const double groom_corners[4][2] = {
            {groom_front, groom_side}, {groom_front, -groom_side},
            {groom_rear, -groom_side}, {groom_rear, groom_side}};
        for (const auto& corner : groom_corners){
            geometry_msgs::msg::Point32 p;
            p.x = static_cast<float>(corner[0]);
            p.y = static_cast<float>(corner[1]);
            groom_polygon.polygon.points.push_back(p);
        }
    }
    groom_footprint_publisher->publish(groom_polygon);

    visualization_msgs::msg::MarkerArray markers;

    // Planner footprint box (robotSizeX/Y, centered on the robot frame).
    visualization_msgs::msg::Marker box;
    box.header.frame_id = robot_frame;
    box.header.stamp = stamp;
    box.ns = "footprint";
    box.id = 0;
    box.type = visualization_msgs::msg::Marker::LINE_STRIP;
    box.action = visualization_msgs::msg::Marker::ADD;
    box.pose.orientation.w = 1.0;
    box.scale.x = 0.05;
    box.color.r = 0.1f;
    box.color.g = 0.8f;
    box.color.b = 0.2f;
    box.color.a = 1.0f;
    box.lifetime = lifetime;
    const double corners[5][2] = {
        {box_offset_x + half_x, half_y}, {box_offset_x + half_x, -half_y},
        {box_offset_x - half_x, -half_y}, {box_offset_x - half_x, half_y},
        {box_offset_x + half_x, half_y}};
    for (const auto& corner : corners){
        geometry_msgs::msg::Point p;
        p.x = corner[0];
        p.y = corner[1];
        p.z = 0.1;
        box.points.push_back(p);
    }
    markers.markers.push_back(box);

    // Measured axle span (the actual wheelbase between wheel centers).
    visualization_msgs::msg::Marker axle;
    axle.header = box.header;
    axle.ns = "wheelbase";
    axle.id = 1;
    axle.type = visualization_msgs::msg::Marker::LINE_STRIP;
    axle.action = visualization_msgs::msg::Marker::ADD;
    axle.pose.orientation.w = 1.0;
    axle.scale.x = 0.08;
    axle.color.r = 0.2f;
    axle.color.g = 0.4f;
    axle.color.b = 1.0f;
    axle.color.a = 1.0f;
    axle.lifetime = lifetime;
    geometry_msgs::msg::Point rear;
    rear.x = envelope.wheel_min_x;
    rear.z = 0.1;
    geometry_msgs::msg::Point front;
    front.x = envelope.wheel_max_x;
    front.z = 0.1;
    axle.points.push_back(rear);
    axle.points.push_back(front);
    markers.markers.push_back(axle);

    // The readout lives in the operator panel (footprint_info topic), not as
    // a 3D label blocking the view of the robot. The DELETE clears the old
    // text marker in rviz sessions that still show one.
    visualization_msgs::msg::Marker text_delete;
    text_delete.header = box.header;
    text_delete.ns = "wheelbase";
    text_delete.id = 2;
    text_delete.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(text_delete);

    std::ostringstream label;
    label.setf(std::ios::fixed);
    label.precision(2);
    label << "wheelbase " << wheelbase << " m | footprint "
          << 2.0 * half_x << " x " << 2.0 * half_y << " m @ x"
          << (box_offset_x >= 0 ? "+" : "") << box_offset_x;
    std_msgs::msg::String info;
    info.data = label.str();
    footprint_info_publisher->publish(info);

    // distToGround visualization: plumb line from base_link to the ASSUMED
    // ground, one disc there, one disc at the ACTUAL map patch under the
    // robot. The vertical gap between the discs is the calibration error;
    // the assumed disc turns amber/red as the gap approaches maxStepHeight
    // (beyond it the next plan returns START_INVALID).
    {
        const double dist_to_ground = get_parameter("distToGround").as_double();
        const double max_step = std::max(1e-6, get_parameter("maxStepHeight").as_double());
        double patch_z = 0.0;
        double pose_z = 0.0;
        std::string height_error;
        bool have_patch = false;
        {
            std::lock_guard<std::recursive_mutex> lock(planner_mutex);
            pose_z = start_pose.pose.position.z;
            have_patch = groundPatchHeightUnderRobot(patch_z, &height_error);
        }
        // Hysteresis: with pose z right at a patch top, the strict below-z
        // rule can alternate found/not-found between ticks, which made the
        // discs blink. Hold the last valid measurement for ~1 s before the
        // markers give up.
        if (have_patch){
            height_last_patch_z_ = patch_z;
            height_miss_streak_ = 0;
        } else if (height_miss_streak_ < 10){
            ++height_miss_streak_;
            if (std::isfinite(height_last_patch_z_)){
                patch_z = height_last_patch_z_;
                have_patch = true;
            }
        }
        const double assumed_rel = -dist_to_ground;          // robot frame
        const double actual_rel = patch_z - pose_z;          // robot frame
        const double gap = std::abs(assumed_rel - actual_rel);
        // Tiny lift so the discs don't z-fight with the coplanar map surface
        // rendering (per-frame shimmer). Falsifies the display by 2 cm only.
        const double draw_lift = 0.02;

        visualization_msgs::msg::Marker plumb;
        plumb.header.frame_id = robot_frame;
        plumb.header.stamp = stamp;
        plumb.ns = "dist_to_ground";
        plumb.id = 0;
        plumb.type = visualization_msgs::msg::Marker::LINE_LIST;
        plumb.action = visualization_msgs::msg::Marker::ADD;
        plumb.pose.orientation.w = 1.0;
        plumb.scale.x = 0.04;
        plumb.color.r = plumb.color.g = plumb.color.b = 0.9f;
        plumb.color.a = 0.9f;
        plumb.lifetime = lifetime;
        geometry_msgs::msg::Point top;
        geometry_msgs::msg::Point bottom;
        bottom.z = assumed_rel;
        plumb.points.push_back(top);
        plumb.points.push_back(bottom);
        markers.markers.push_back(plumb);

        visualization_msgs::msg::Marker assumed;
        assumed.header = plumb.header;
        assumed.ns = "dist_to_ground";
        assumed.id = 1;
        assumed.type = visualization_msgs::msg::Marker::CYLINDER;
        assumed.action = visualization_msgs::msg::Marker::ADD;
        assumed.pose.orientation.w = 1.0;
        assumed.pose.position.z = assumed_rel + draw_lift;
        assumed.scale.x = 1.0;
        assumed.scale.y = 1.0;
        assumed.scale.z = 0.03;
        if (!have_patch){
            assumed.color.r = assumed.color.g = assumed.color.b = 0.6f;
        } else if (gap > max_step){
            assumed.color.r = 1.0f; assumed.color.g = 0.15f; assumed.color.b = 0.15f;
        } else if (gap > 0.5 * max_step){
            assumed.color.r = 1.0f; assumed.color.g = 0.65f; assumed.color.b = 0.0f;
        } else {
            assumed.color.r = 0.2f; assumed.color.g = 0.9f; assumed.color.b = 0.3f;
        }
        assumed.color.a = 0.85f;
        assumed.lifetime = lifetime;
        markers.markers.push_back(assumed);

        visualization_msgs::msg::Marker actual;
        actual.header = plumb.header;
        actual.ns = "dist_to_ground";
        actual.id = 2;
        actual.type = visualization_msgs::msg::Marker::CYLINDER;
        actual.action = have_patch ? visualization_msgs::msg::Marker::ADD
                                   : visualization_msgs::msg::Marker::DELETE;
        actual.pose.orientation.w = 1.0;
        actual.pose.position.z = actual_rel + draw_lift;
        actual.scale.x = 1.3;
        actual.scale.y = 1.3;
        actual.scale.z = 0.03;
        actual.color.r = 0.25f;
        actual.color.g = 0.55f;
        actual.color.b = 1.0f;
        actual.color.a = 0.6f;
        actual.lifetime = lifetime;
        markers.markers.push_back(actual);

        // Numbers for the operator panel at ~1 Hz (this timer runs at 10 Hz).
        if (++height_info_tick_ >= 10){
            height_info_tick_ = 0;
            std::ostringstream info;
            info.setf(std::ios::fixed);
            info.precision(2);
            info << "distToGround " << dist_to_ground << " m";
            if (have_patch){
                const double delta = assumed_rel - actual_rel;
                info << " | map \u0394 " << (delta >= 0 ? "+" : "") << delta << " m ";
                if (gap > max_step) info << "(START WOULD BE INVALID)";
                else if (gap > 0.5 * max_step) info << "(check height)";
                else info << "(ok)";
            } else {
                info << " | map: " << height_error;
            }
            std_msgs::msg::String info_msg;
            info_msg.data = info.str();
            height_info_publisher->publish(info_msg);
        }
    }

    footprint_marker_publisher->publish(markers);
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

void PathPlannerNode::planReturnCurrentCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response){
    std::lock_guard<std::recursive_mutex> lock(planner_mutex);
    if (!has_last_mission_start || !got_map || !is_configured){
        response->success = false;
        response->message = "Cannot plan direct return: mission start or map is unavailable.";
        return;
    }
    if (!get_parameter("read_pose_from_topic").as_bool() && !updatePoseFromTF()){
        response->success = false;
        response->message = "Cannot plan direct return: current robot pose is unavailable.";
        return;
    }
    const auto saved_start = start_pose_rbs;
    const auto saved_goal = goal_pose_rbs;
    start_pose_rbs.position = Eigen::Vector3d(
        start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z);
    start_pose_rbs.orientation = Eigen::Quaterniond(
        start_pose.pose.orientation.w, start_pose.pose.orientation.x,
        start_pose.pose.orientation.y, start_pose.pose.orientation.z);
    goal_pose_rbs = last_mission_start;
    is_return_plan = true;
    plan(false);
    is_return_plan = false;
    start_pose_rbs = saved_start;
    goal_pose_rbs = saved_goal;
    response->success = latest_planning_result == ugv_nav4d::Planner::FOUND_SOLUTION;
    response->message = response->success
        ? "Direct return from current pose is ready; review and Execute."
        : std::string("Direct return planning failed: ") + planningResultToString(latest_planning_result);
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
        if (zone.zone_type == ugv_nav4d_ros2::msg::ForbiddenZone::CAUTION){ outline.color.g = 0.75; }
        else if (zone.zone_type == ugv_nav4d_ros2::msg::ForbiddenZone::SPEED_LIMIT){ outline.color.r = 0.1; outline.color.g = 0.5; outline.color.b = 1.0; }
        else if (zone.zone_type == ugv_nav4d_ros2::msg::ForbiddenZone::PREFERRED){ outline.color.r = 0.1; outline.color.g = 1.0; outline.color.b = 0.2; }
        else if (zone.zone_type == ugv_nav4d_ros2::msg::ForbiddenZone::DIRECTION_RESTRICTED){ outline.color.r = 0.7; outline.color.g = 0.2; outline.color.b = 1.0; }
        else if (zone.zone_type == ugv_nav4d_ros2::msg::ForbiddenZone::ANNOTATION){ outline.color.r = 1.0; outline.color.g = 1.0; outline.color.b = 1.0; }
        else if (zone.zone_type == ugv_nav4d_ros2::msg::ForbiddenZone::TRAVERSABLE){ outline.color.r = 0.3; outline.color.g = 1.0; outline.color.b = 0.7; }
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
        wall.color = outline.color;
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
        static const std::array<const char*, 7> type_names{
            "KEEP-OUT", "CAUTION", "SPEED", "PREFERRED", "DIRECTION", "NOTE", "TRAVERSABLE"};
        const std::string type_name = zone.zone_type < type_names.size()
            ? type_names[zone.zone_type] : "ZONE";
        if (zone.zone_type == ugv_nav4d_ros2::msg::ForbiddenZone::ANNOTATION &&
            !zone.label.empty()){
            // An annotation IS its text; prefixing "NOTE:" was just clutter.
            label.text = std::to_string(i + 1) + ": " + zone.label;
        } else {
            label.text = std::to_string(i + 1) + " " + type_name;
            if (!zone.label.empty()) label.text += ": " + zone.label;
        }
        marker_array.markers.push_back(label);
    }

    forbidden_zone_marker_publisher->publish(marker_array);
    ugv_nav4d_ros2::msg::OperationalZoneArray zones_message;
    zones_message.header.frame_id = frame;
    zones_message.header.stamp = now;
    zones_message.zones = forbidden_zones;
    operational_zones_publisher->publish(zones_message);
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
            const Eigen::Vector3f ply_offset(
                static_cast<float>(get_parameter("mls_file_offset_x").as_double()),
                static_cast<float>(get_parameter("mls_file_offset_y").as_double()),
                static_cast<float>(get_parameter("mls_file_offset_z").as_double()));
            if (ply_offset.norm() > 1e-9f){
                for (auto& pt : cloud->points){
                    pt.x += ply_offset.x();
                    pt.y += ply_offset.y();
                    pt.z += ply_offset.z();
                }
                RCLCPP_INFO_STREAM(this->get_logger(), "Translated loaded PLY cloud by ("
                                   << ply_offset.transpose() << ").");
            }
            pcl::PointXYZ min, max; 
            pcl::getMinMax3D (*cloud, min, max); 

            const double size_x = max.x - min.x;
            const double size_y = max.y - min.y;

            mls_min_x = min.x;
            mls_min_y = min.y;
            {
                // Same placement + suggestion report as the .bin loader; the
                // bounds are measured after the offset was applied, so the
                // suggested values are absolute, not incremental.
                const double applied_x = get_parameter("mls_file_offset_x").as_double();
                const double applied_y = get_parameter("mls_file_offset_y").as_double();
                RCLCPP_INFO_STREAM(this->get_logger(), std::fixed << std::setprecision(1)
                                   << "Loaded PLY '" << path << "': spans x [" << min.x
                                   << ", " << max.x << "], y [" << min.y << ", " << max.y
                                   << "] in the world frame. To center it on the origin set "
                                   << "mls_file_offset_x: " << applied_x - (min.x + max.x) / 2.0
                                   << ", mls_file_offset_y: " << applied_y - (min.y + max.y) / 2.0
                                   << " (offsets are absolute, not incremental).");
            }

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
            return true;
        }
        // A failed PLY read must NOT report success: mls_map_ptr would stay null
        // and every consumer (initializeMLSMap, configurePlanner, publishMLSMap)
        // would dereference it.
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to read PLY '" << path << "'.");
        return false;
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
    try
    {
        std::lock_guard<std::recursive_mutex> lock(planner_mutex);
        if (!mls_map_ptr)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot save MLS map: no map loaded.");
            return false;
        }
        boost::archive::binary_oarchive archive(binFile);
        archive << *mls_map_ptr;
    }
    catch (const std::exception& ex)
    {
        // boost::archive throws on stream failure (disk full, permissions); an
        // uncaught throw here would terminate the node from the action thread.
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to save MLS map to '"
                            << fileToUse << "': " << ex.what());
        return false;
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

        const Eigen::Vector3d offset(get_parameter("mls_file_offset_x").as_double(),
                                     get_parameter("mls_file_offset_y").as_double(),
                                     get_parameter("mls_file_offset_z").as_double());
        if (offset.norm() > 1e-9){
            // world = localFrame^-1 * grid, so shifting the map by +offset in
            // the world means composing the local frame with -offset.
            mls_map_ptr->getLocalFrame() = mls_map_ptr->getLocalFrame() *
                                           Eigen::Translation3d(-offset);
            RCLCPP_INFO_STREAM(this->get_logger(), "Translated loaded MLS by ("
                               << offset.transpose() << ").");
        }
        // The grid's world-frame corner; .bin maps carry their own placement,
        // so the dist_min_x/y parameters must not be used for index math.
        const Eigen::Vector3d world_corner =
            mls_map_ptr->getLocalFrame().inverse(Eigen::Isometry) * Eigen::Vector3d::Zero();
        mls_min_x = world_corner.x();
        mls_min_y = world_corner.y();
        const auto num_cells = mls_map_ptr->getNumCells();
        const auto res = mls_map_ptr->getResolution();
        const double span_max_x = mls_min_x + num_cells.x() * res.x();
        const double span_max_y = mls_min_y + num_cells.y() * res.y();
        const double applied_x = get_parameter("mls_file_offset_x").as_double();
        const double applied_y = get_parameter("mls_file_offset_y").as_double();
        RCLCPP_INFO_STREAM(this->get_logger(), std::fixed << std::setprecision(1)
                           << "Loaded MLS Map from " << filename
                           << ": spans x [" << mls_min_x << ", " << span_max_x
                           << "], y [" << mls_min_y << ", " << span_max_y
                           << "] in the world frame. To center it on the origin set "
                           << "mls_file_offset_x: " << applied_x - (mls_min_x + span_max_x) / 2.0
                           << ", mls_file_offset_y: " << applied_y - (mls_min_y + span_max_y) / 2.0
                           << " (offsets are absolute, not incremental).");
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

    // Anchor recording is deferred to executePathCallback: a plan is only a
    // preview, and a discarded preview must not move the return-home anchor.
    pending_records_mission = record_mission;
    if (record_mission){
        pending_mission_start = start_pose_rbs;
        pending_mission_goal = goal_pose_rbs;
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
    visualization_msgs::msg::MarkerArray preview_colored_message;
    auto now = this->get_clock()->now();
    const std::string world_frame = get_parameter("world_frame").as_string();

    visualization_msgs::msg::Marker clear_colored_path;
    clear_colored_path.action = visualization_msgs::msg::Marker::DELETEALL;
    colored_path_message.markers.push_back(clear_colored_path);
    preview_colored_message.markers.push_back(clear_colored_path);

    nav_msgs::msg::Path path;
    path.header.frame_id = world_frame;

    nav_msgs::msg::Path path_segment;
    std::string label_last;
    std::string label;
    bool first_segment = true;
    const bool is_recovery_path = std::any_of(
        trajectory3D.begin(), trajectory3D.end(),
        [](const auto& trajectory){
            return trajectory.kind == trajectory_follower::TRAJECTORY_KIND_RESCUE;
        });

    if (found_solution) {
        for (size_t seg_idx = 0; seg_idx < trajectory3D.size(); ++seg_idx) {
            auto& trajectory = trajectory3D[seg_idx];
            const bool is_rescue =
                trajectory.kind == trajectory_follower::TRAJECTORY_KIND_RESCUE;

            // Assign label based on current segment
            if (is_rescue && trajectory.speed > 0) {
                label = "Recovery forward";
            } else if (is_rescue && trajectory.speed < 0) {
                label = "Recovery backward";
            } else if (trajectory.driveMode == trajectory_follower::DriveMode::ModeAckermann && trajectory.speed > 0) {
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
                route_line.ns = is_rescue ? "recovery_route" :
                    (is_return_plan ? "return_route" : "outward_route");
                route_line.id = static_cast<int>(2 * seg_idx);
                route_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
                route_line.action = visualization_msgs::msg::Marker::ADD;
                route_line.pose.orientation.w = 1.0;
                route_line.scale.x = 0.22;
                route_line.color.a = 0.65;
                if (is_rescue) {
                    route_line.color.r = 0.75;
                    route_line.color.g = 0.15;
                    route_line.color.b = 1.0;
                } else if (is_return_plan) {
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
                if (is_rescue) {
                    motion_line.color.r = 0.95;
                    motion_line.color.g = 0.25;
                    motion_line.color.b = 1.0;
                } else if (label == "Forward") {
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

                // Preview variant: same geometry, unmistakably different look
                // (magenta underlay, muted motion overlay) so the operator can
                // tell the preview from the executing route. Magenta appears in
                // no travmap patch type, no MLS colormap and no other path
                // color, and it keeps contrast on the green traversable
                // patches where white/yellow wash out.
                visualization_msgs::msg::Marker preview_route = route_line;
                preview_route.ns = "preview_route";
                preview_route.color.r = 1.0;
                preview_route.color.g = 0.0;
                preview_route.color.b = 0.9;
                preview_route.color.a = 0.6;
                visualization_msgs::msg::Marker preview_motion = motion_line;
                preview_motion.ns = "preview_motion";
                preview_motion.color.a = 0.8;
                preview_colored_message.markers.push_back(preview_route);
                preview_colored_message.markers.push_back(preview_motion);
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

                    // A degenerate end tangent normalizes to NaN and would append
                    // NaN poses that Nav2's controller consumes blindly.
                    if (end_tangent.norm() < 1e-9 || !end_tangent.allFinite()) {
                        RCLCPP_WARN_STREAM(this->get_logger(),
                            "Skipping trajectory extension: degenerate spline end tangent.");
                        continue;
                    }
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

                    // Draw the extension distinctly: the controller follows
                    // these poses (goal-tolerance runway), so the deviation
                    // anchor must land on VISIBLE geometry instead of what
                    // looked like empty space next to the colored route.
                    visualization_msgs::msg::Marker ext_line;
                    ext_line.header.frame_id = get_parameter("world_frame").as_string();
                    ext_line.header.stamp = now;
                    ext_line.ns = "trajectory_extension";
                    ext_line.id = static_cast<int>(seg_idx);
                    ext_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
                    ext_line.action = visualization_msgs::msg::Marker::ADD;
                    ext_line.pose.orientation.w = 1.0;
                    ext_line.scale.x = 0.08;
                    ext_line.color.r = 0.75f;
                    ext_line.color.g = 0.75f;
                    ext_line.color.b = 0.78f;
                    ext_line.color.a = 0.9f;
                    geometry_msgs::msg::Point ext_p0;
                    ext_p0.x = end_point.x(); ext_p0.y = end_point.y(); ext_p0.z = end_point.z();
                    geometry_msgs::msg::Point ext_p1;
                    ext_p1.x = ext_point_half.x(); ext_p1.y = ext_point_half.y(); ext_p1.z = ext_point_half.z();
                    geometry_msgs::msg::Point ext_p2;
                    ext_p2.x = ext_point_full.x(); ext_p2.y = ext_point_full.y(); ext_p2.z = ext_point_full.z();
                    ext_line.points.push_back(ext_p0);
                    ext_line.points.push_back(ext_p1);
                    ext_line.points.push_back(ext_p2);
                    colored_path_message.markers.push_back(ext_line);
                    visualization_msgs::msg::Marker preview_ext = ext_line;
                    preview_ext.ns = "preview_extension";
                    preview_ext.color.a = 0.5f;
                    preview_colored_message.markers.push_back(preview_ext);
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

    // Publish unconditionally: on failure this clears the previously published
    // PREVIEW instead of leaving a stale trajectory on the topic. Only the preview
    // topics are touched here — the executing-route displays (path/mission_path)
    // keep showing what the follower is driving until Execute promotes this plan.
    path.header.stamp = now;
    preview_path_publisher->publish(path);
    labeled_path_publisher->publish(labeled_path_message);
    preview_colored_path_publisher->publish(preview_colored_message);
    pending_display_path = path;
    pending_display_markers = colored_path_message;
    publishRouteRisk(path, labeled_path_message);

    // Hold the path for the execution gate. Anything that plans (or fails to)
    // replaces the pending path, so Execute always sends what RViz shows.
    pending_labeled_path = labeled_path_message;
    has_pending_path = found_solution && !labeled_path_message.paths.empty();
    path_approved = false;
    publishPreviewPending();
    pending_path_is_recovery = has_pending_path && is_recovery_path;
    // Deliberately do NOT lower route_valid here: a new preview says nothing about
    // the route the follower is currently driving. Lowering it made every mid-drive
    // goal click pause the robot. The preview itself is protected by the execute
    // gate (path_approved); route_valid is only lowered when the executing route is
    // genuinely invalidated (map regeneration, zone changes).
    if (has_pending_path) {
        publishStatus("Path ready (" + std::to_string(pending_labeled_path.paths.size()) +
                      " segment(s)) - review in RViz, then Execute to send to the follower");
    }
}

void PathPlannerNode::publishRouteRisk(
        const nav_msgs::msg::Path& path,
        const ugv_nav4d_ros2::msg::LabeledPathArray& labeled){
    if (!route_risk_publisher){
        return;
    }
    ugv_nav4d_ros2::msg::RouteRisk risk;
    risk.header = path.header;
    risk.minimum_clearance = -1.0f;
    risk.minimum_orientation_margin = static_cast<float>(2.0 * M_PI);
    if (path.poses.empty() || !traversability_generator_ptr){
        risk.valid = false;
        risk.summary = "No valid route available";
        route_risk_publisher->publish(risk);
        return;
    }

    for (size_t i = 1; i < path.poses.size(); ++i){
        const auto& a = path.poses[i - 1].pose.position;
        const auto& b = path.poses[i].pose.position;
        const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
        risk.path_length += std::sqrt(dx * dx + dy * dy + dz * dz);
        risk.max_step = std::max(risk.max_step, static_cast<float>(std::abs(dz)));
    }
    for (size_t i = 0; i < labeled.paths.size() && i < labeled.labels.size(); ++i){
        double length = 0.0;
        const auto& poses = labeled.paths[i].poses;
        for (size_t k = 1; k < poses.size(); ++k){
            const auto& a = poses[k - 1].pose.position;
            const auto& b = poses[k].pose.position;
            length += std::hypot(b.x - a.x, b.y - a.y);
        }
        if (labeled.labels[i] == "Backward") risk.reverse_distance += length;
        else risk.forward_distance += length;
    }
    const double speed = std::max(0.01, mobility_config.translationSpeed);
    risk.estimated_duration = risk.path_length / speed;

    const auto& map = traversability_generator_ptr->getTraversabilityMap();
    std::vector<Eigen::Vector2d> obstacles;
    for (const auto& levels : map){
        for (const auto* node : levels){
            if (node->getUserData().nodeType == traversability_generator3d::NodeType::OBSTACLE ||
                node->getUserData().nodeType == traversability_generator3d::NodeType::INFLATED_OBSTACLE){
                Eigen::Vector3d p;
                if (obstacles.size() < 20000 &&
                    map.fromGrid(node->getIndex(), p, node->getHeight(), false)){
                    obstacles.emplace_back(p.x(), p.y());
                }
            }
        }
    }

    size_t known_samples = 0, unknown_samples = 0, partial_samples = 0;
    double minimum_clearance = std::numeric_limits<double>::max();
    const size_t stride = std::max<size_t>(1, path.poses.size() / 200);
    for (size_t i = 0; i < path.poses.size(); i += stride){
        const auto& pose = path.poses[i].pose.position;
        const Eigen::Vector3d query(pose.x, pose.y, pose.z);
        maps::grid::Index index;
        if (map.toGrid(query, index)){
            const traversability_generator3d::TravGenNode* best = nullptr;
            double best_dz = std::numeric_limits<double>::max();
            for (const auto* node : map.at(index)){
                const double dz = std::abs(node->getHeight() - pose.z);
                if (dz < best_dz){ best = node; best_dz = dz; }
            }
            if (best){
                ++known_samples;
                const auto& data = best->getUserData();
                risk.max_slope = std::max(risk.max_slope, static_cast<float>(data.slope));
                if (data.nodeType == traversability_generator3d::NodeType::UNKNOWN) ++unknown_samples;
                if (data.nodeType == traversability_generator3d::NodeType::PARTIALLY_TRAVERSABLE) ++partial_samples;
                if (!data.allowedOrientations.empty()){
                    double widest = 0.0;
                    for (const auto& allowed : data.allowedOrientations){
                        widest = std::max(widest, allowed.getWidth());
                    }
                    risk.minimum_orientation_margin = std::min(
                        risk.minimum_orientation_margin, static_cast<float>(widest));
                }
            } else {
                ++unknown_samples;
            }
        } else {
            ++unknown_samples;
        }
        for (const auto& obstacle : obstacles){
            minimum_clearance = std::min(
                minimum_clearance, (obstacle - Eigen::Vector2d(pose.x, pose.y)).norm());
        }
    }
    const double samples = static_cast<double>(known_samples + unknown_samples);
    if (samples > 0.0){
        risk.unknown_percentage = 100.0 * unknown_samples / samples;
        risk.partial_percentage = 100.0 * partial_samples / samples;
    }
    if (std::isfinite(minimum_clearance)) risk.minimum_clearance = minimum_clearance;
    if (risk.minimum_orientation_margin >= 2.0 * M_PI - 1e-3)
        risk.minimum_orientation_margin = static_cast<float>(2.0 * M_PI);

    if (risk.max_slope > 0.8 * traversability_config.maxSlope)
        risk.warnings.push_back("Route approaches the configured maximum slope");
    if (risk.unknown_percentage > 0.0)
        risk.warnings.push_back("Route crosses unknown terrain");
    if (risk.minimum_clearance >= 0.0 &&
        risk.minimum_clearance < std::hypot(
            traversability_config.robotSizeX + 2.0 * std::abs(traversability_config.footprintOffsetX),
            traversability_config.robotSizeY) / 2.0)
        risk.warnings.push_back("Route has low obstacle clearance");
    risk.high_risk_sections = static_cast<uint32_t>(risk.warnings.size());
    risk.valid = true;
    std::ostringstream summary;
    summary << std::fixed << std::setprecision(1) << risk.path_length << " m, "
            << risk.estimated_duration << " s, max slope "
            << (risk.max_slope * 180.0 / M_PI) << " deg";
    if (!risk.warnings.empty()) summary << ", " << risk.warnings.size() << " warning(s)";
    risk.summary = summary.str();
    route_risk_publisher->publish(risk);
}

void PathPlannerNode::inspectOrientationsCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg){
    //try_to_lock: this is a purely informational query. Blocking here while a
    //long plan holds the mutex would silently stall every service behind this
    //callback for up to maxTime seconds.
    std::unique_lock<std::recursive_mutex> lock(planner_mutex, std::try_to_lock);
    if (!lock.owns_lock()){
        publishStatus("Orientation inspection: planner is busy, try again in a moment.");
        return;
    }

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    //An empty region is the explicit CLEAR request from the tool.
    if (msg->polygon.points.size() < 3){
        allowed_orientation_marker_publisher->publish(markers);
        publishStatus("Orientation markers cleared.");
        return;
    }
    const std::string world_frame = normalizedFrame(get_parameter("world_frame").as_string());
    const std::string region_frame = normalizedFrame(msg->header.frame_id);
    if (!region_frame.empty() && region_frame != world_frame){
        publishStatus("Orientation inspection rejected: RViz fixed frame must match " + world_frame);
        return;
    }
    if (!traversability_generator_ptr || !got_map){
        publishStatus("Orientation inspection: traversability map is unavailable.");
        return;
    }

    //Region polygon (XY) + reference height from the clicked vertices, so the
    //inspection stays on the storey the operator clicked on.
    std::vector<geometry_msgs::msg::Point> polygon;
    double ref_height = 0.0;
    for (const auto& p : msg->polygon.points){
        geometry_msgs::msg::Point q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        polygon.push_back(q);
        ref_height += p.z;
    }
    ref_height /= static_cast<double>(polygon.size());

    const auto& trav_map_3d = traversability_generator_ptr->getTraversabilityMap();
    const double wedge_radius = 0.45 * get_parameter("grid_resolution").as_double();
    constexpr size_t MAX_CELLS = 2000; //marker-count guard for huge selections

    visualization_msgs::msg::Marker wedges;
    wedges.header.frame_id = get_parameter("world_frame").as_string();
    wedges.header.stamp = this->get_clock()->now();
    wedges.ns = "allowed_orientations";
    wedges.id = 0;
    wedges.type = visualization_msgs::msg::Marker::LINE_LIST;
    wedges.action = visualization_msgs::msg::Marker::ADD;
    wedges.pose.orientation.w = 1.0;
    wedges.scale.x = 0.03;
    wedges.color.r = 0.2;
    wedges.color.g = 1.0;
    wedges.color.b = 0.3;
    wedges.color.a = 1.0;

    size_t cells = 0;
    bool truncated = false;
    for(const maps::grid::LevelList<traversability_generator3d::TravGenNode*>& level : trav_map_3d)
    {
        for(const traversability_generator3d::TravGenNode* node : level)
        {
            if (node->getUserData().nodeType != traversability_generator3d::NodeType::PARTIALLY_TRAVERSABLE){
                continue;
            }
            if (std::abs(node->getHeight() - ref_height) > traversability_config.robotHeight){
                continue; //other storey
            }
            Eigen::Vector3d position;
            trav_map_3d.fromGrid(node->getIndex(), position, node->getHeight(), true);
            if (!pointInPolygonXY(position.x(), position.y(), polygon)){
                continue;
            }
            if (cells >= MAX_CELLS){
                truncated = true;
                break;
            }
            ++cells;

            geometry_msgs::msg::Point center;
            center.x = position.x();
            center.y = position.y();
            center.z = position.z() + 0.08;

            //One wedge per allowed interval: two radial edges + an arc,
            //as LINE_LIST segment pairs.
            for (const auto& allowed : node->getUserData().allowedOrientations){
                const double start = allowed.getStart().getRad();
                const double width = allowed.getWidth();
                const int arc_steps = std::max(2, static_cast<int>(std::ceil(width / 0.35)));
                geometry_msgs::msg::Point prev;
                for (int k = 0; k <= arc_steps; ++k){
                    const double angle = start + width * static_cast<double>(k) / arc_steps;
                    geometry_msgs::msg::Point rim;
                    rim.x = center.x + wedge_radius * std::cos(angle);
                    rim.y = center.y + wedge_radius * std::sin(angle);
                    rim.z = center.z;
                    if (k == 0 || k == arc_steps){
                        //radial edge marking the interval boundary
                        wedges.points.push_back(center);
                        wedges.points.push_back(rim);
                    }
                    if (k > 0){
                        wedges.points.push_back(prev);
                        wedges.points.push_back(rim);
                    }
                    prev = rim;
                }
            }
        }
        if (truncated){
            break;
        }
    }

    if (!wedges.points.empty()){
        markers.markers.push_back(wedges);
    }
    allowed_orientation_marker_publisher->publish(markers);
    std::ostringstream status;
    status << "Orientation inspection: " << cells << " partially traversable cell(s) in the region";
    if (truncated){
        status << " (display capped at " << MAX_CELLS << ")";
    }
    if (cells == 0){
        status << " - nothing to show";
    }
    status << ".";
    publishStatus(status.str());
}

void PathPlannerNode::clearExecutingPathDisplay(){
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = get_parameter("world_frame").as_string();
    empty_path.header.stamp = this->get_clock()->now();
    combined_path_publisher->publish(empty_path);
    // Also clear the per-pose direction arrows: visualize_path.py treats an
    // empty LabeledPathArray on the DISPLAY topic as "clear". The follower's
    // execute_path_segments input is deliberately not touched.
    ugv_nav4d_ros2::msg::LabeledPathArray empty_labeled;
    labeled_path_publisher->publish(empty_labeled);
    visualization_msgs::msg::MarkerArray clear_markers;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    clear_markers.markers.push_back(clear_marker);
    colored_path_publisher->publish(clear_markers);
}

bool PathPlannerNode::validatePendingPath(){
    if (!has_pending_path || !traversability_generator_ptr){
        return false;
    }
    const auto& map = traversability_generator_ptr->getTraversabilityMap();
    bool valid = true;
    for (size_t segment_index = 0; segment_index < pending_labeled_path.paths.size(); ++segment_index){
        const auto& segment = pending_labeled_path.paths[segment_index];
        for (size_t pose_index = 0; pose_index < segment.poses.size(); ++pose_index){
            // A native rescue motion is intentionally allowed to overlap the
            // obstacle/frontier around the stuck robot. Its contract is that
            // the endpoint is free, so map refreshes validate that endpoint.
            if (pending_path_is_recovery &&
                (segment_index + 1 != pending_labeled_path.paths.size() ||
                 pose_index + 1 != segment.poses.size())){
                continue;
            }
            const auto& stamped_pose = segment.poses[pose_index];
            const auto& p = stamped_pose.pose.position;
            maps::grid::Index index;
            if (!map.toGrid(Eigen::Vector3d(p.x, p.y, p.z), index)){
                valid = false;
                break;
            }
            const traversability_generator3d::TravGenNode* best = nullptr;
            double best_dz = std::numeric_limits<double>::max();
            for (const auto* node : map.at(index)){
                const double dz = std::abs(node->getHeight() - p.z);
                if (dz < best_dz){ best = node; best_dz = dz; }
            }
            if (!best || best->getUserData().nodeType == traversability_generator3d::NodeType::OBSTACLE ||
                best->getUserData().nodeType == traversability_generator3d::NodeType::INFLATED_OBSTACLE){
                valid = false;
                break;
            }
        }
        if (!valid) break;
    }
    // route_valid describes the route the follower is driving. Only the approved
    // path is that route; a pending unapproved preview must not raise or lower the
    // flag (lowering it would pause an executing mission over a stale preview).
    if (path_approved){
        std_msgs::msg::Bool route_valid;
        route_valid.data = valid;
        route_valid_publisher->publish(route_valid);
    }
    if (!valid){
        const bool was_approved = path_approved;
        has_pending_path = false;
        path_approved = false;
        publishPreviewPending();
        pending_path_is_recovery = false;
        if (was_approved){
            clearExecutingPathDisplay();
            publishMissionStatus(ugv_nav4d_ros2::msg::MissionStatus::PAUSED,
                                 "Remaining route is no longer traversable; replan required");
        } else {
            publishStatus("Planned preview is no longer traversable; replan required.");
        }
    }
    return valid;
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
    path_approved = true;
    publishPreviewPending();
    // Commit the mission anchor now that the plan is actually being driven; a
    // preview that was never executed leaves the return-home anchor untouched.
    if (pending_records_mission){
        last_mission_start = pending_mission_start;
        last_mission_goal = pending_mission_goal;
        has_last_mission_start = true;
        pending_records_mission = false;
    }
    std_msgs::msg::Bool route_valid;
    route_valid.data = true;
    route_valid_publisher->publish(route_valid);
    // Promote the preview to the executing displays and clear the preview topics.
    combined_path_publisher->publish(pending_display_path);
    colored_path_publisher->publish(pending_display_markers);
    nav_msgs::msg::Path empty_preview;
    empty_preview.header.frame_id = get_parameter("world_frame").as_string();
    empty_preview.header.stamp = this->get_clock()->now();
    preview_path_publisher->publish(empty_preview);
    visualization_msgs::msg::MarkerArray clear_preview;
    visualization_msgs::msg::Marker clear_preview_marker;
    clear_preview_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    clear_preview.markers.push_back(clear_preview_marker);
    preview_colored_path_publisher->publish(clear_preview);
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
    path_approved = false;
    publishPreviewPending();
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
    // Translation applied to a file-loaded map: maps recorded on the robot sit
    // at their recording-time world coordinates (e.g. UTM); this pulls them to
    // the current session's origin. Applied to .bin and .ply alike.
    declare_parameter("mls_file_offset_x", 0.0, param_desc);
    declare_parameter("mls_file_offset_y", 0.0, param_desc);
    declare_parameter("mls_file_offset_z", 0.0, param_desc);
    // Grooming: live-tunable from the operator panel, so NOT the read-only
    // descriptor the file parameters above use.
    {
        rcl_interfaces::msg::ParameterDescriptor groom_desc;
        groom_desc.description =
            "Grooming knob; read on every groom tick, never triggers a rebuild.";
        // Patches higher than groom_delete_top above the measured ground
        // survive the under-robot delete (structures the robot drives beneath).
        declare_parameter("groom_delete_top", 2.0, groom_desc);
        declare_parameter("groom_margin", 0.2, groom_desc);
    }
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
    declare_parameter("mission_file", "ugv_nav4d_mission.txt");
    declare_parameter("speed_zone_surface_tolerance", 0.75);
    declare_parameter("speed_zone_release_delay", 0.5);
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
    declare_parameter("reedsSheppMaxCusps", 1); // max direction changes per accepted RS curve; < 0 = unlimited

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
    // Forward offset of the footprint-box center from the robot frame (m):
    // base_link is not at the machine center (tool side is longer). 0 =
    // legacy centered box. NOTE: no "footprint_" underscore prefix on
    // purpose -- changing it must trigger a planner rebuild.
    declare_parameter("footprintOffsetX", 0.0);
    declare_parameter("slopeMetric", "NONE"); // Options: NONE, AVG_SLOPE, MAX_SLOPE, TRIANGLE_SLOPE
    declare_parameter("slopeMetricScale", 1.0);
    declare_parameter("obstacleInflationMultiplier", 1.0);
    declare_parameter("partiallyTraversableMultiplier", 2.0);
    declare_parameter("numYawSamples", 12); // yaw samples over [0,180) for partial-cell orientations
    declare_parameter("extend_trajectory", false);
    declare_parameter("extension_distance", 0.0);

    // Footprint update from the live robot geometry (variable wheelbase):
    // the update_footprint service measures the wheel-link envelope via TF
    // and applies robotSize = 2 * (max wheel offset + margin) per axis. The
    // margins cover the body/tool overhang beyond the wheel centers and the
    // wheel dimensions themselves; calibrate them so the result matches the
    // measured machine at a known wheelbase.
    declare_parameter("footprint_wheel_frames", std::vector<std::string>{
        "arter/wheel_fl_link", "arter/wheel_fr_link",
        "arter/wheel_rl_link", "arter/wheel_rr_link"});
    // Additional frames included in the footprint envelope but not in the
    // wheelbase: the tool mount covers the manipulator/shovel at its LIVE
    // pose. All tools attach under tool_link (see attach_tool.xacro).
    declare_parameter("footprint_extra_frames", std::vector<std::string>{
        "arter/tool_link"});
    // Margins beyond the measured frames, per side: front = shovel/tool extent
    // beyond tool_link, rear = body extent beyond the rear wheel centers.
    // Calibrate so the published polygon hugs the real machine.
    declare_parameter("footprint_margin_front", 1.1);
    declare_parameter("footprint_margin_rear", 0.6);
    declare_parameter("footprint_margin_y", 0.65);
    // Period [s] for publishing the measured wheelbase (Float32), the
    // footprint polygon and the RViz markers; <= 0 disables the publisher.
    declare_parameter("footprint_publish_period", 0.1);
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
    traversability_config.footprintOffsetX          = get_parameter("footprintOffsetX").as_double();
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
    // One thread knob for the whole node: the traversability generator's own
    // numThreads is overwritten with the planner's value (applied by travgen at
    // the start of every expandAll).
    traversability_config.numThreads                = planner_config.numThreads;
#ifdef _OPENMP
    // One thread budget for the whole node: the traversability generator's
    // wave-parallel expansion runs OpenMP regions BEFORE the first plan, but
    // ugv_nav4d only applies numThreads at Planner::plan(). Set it here so map
    // generation is capped by the same parameter instead of grabbing all cores.
    omp_set_num_threads(std::max(1, static_cast<int>(planner_config.numThreads)));
#endif
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
    planner_config.reedsSheppMaxCusps               = get_parameter("reedsSheppMaxCusps").as_int();
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
        rclcpp::Parameter("footprintOffsetX", trav.footprintOffsetX),
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
        rclcpp::Parameter("reedsSheppMaxCusps", planner.reedsSheppMaxCusps),
    };

    // Setting the parameters triggers parametersCallback(), which queues them so the
    // parameterUpdateTimerCallback() rebuilds the planner via configurePlanner().
    set_parameters(params);
    RCLCPP_INFO(this->get_logger(), "Applied %zu parameters from GUI; planner will reconfigure.", params.size());
}

void PathPlannerNode::publishRebuilding(bool active){
    rebuild_depth_ = std::max(0, rebuild_depth_ + (active ? 1 : -1));
    std_msgs::msg::Bool msg;
    msg.data = rebuild_depth_ > 0;
    rebuilding_publisher->publish(msg);
}

void PathPlannerNode::configurePlanner(){
    // Latched busy flag: the panel greys its buttons while this runs (map
    // regeneration + environment rebuild can take a long time).
    ScopedRebuildFlag rebuild_flag(*this);
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
        const auto t_expand = std::chrono::steady_clock::now();
        traversability_generator_ptr->expandAll(startPosition);
        RCLCPP_INFO_STREAM(this->get_logger(), "Timing: trav-map expansion took "
            << std::chrono::duration<double>(std::chrono::steady_clock::now() - t_expand).count()
            << " s.");
        applyForbiddenZones();
        rebuildSpeedZoneCache();
        auto travMap = traversability_generator_ptr->getTraversabilityMap();
        // planner_ptr was reset above, so this updateMap always rebuilds the
        // environment and regenerates the motion primitives.
        const auto t_env = std::chrono::steady_clock::now();
        planner_ptr->updateMap(travMap);
        RCLCPP_INFO_STREAM(this->get_logger(),
            "Timing: environment build (incl. motion-primitive generation) took "
            << std::chrono::duration<double>(std::chrono::steady_clock::now() - t_env).count()
            << " s.");
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
    const auto t_publish = std::chrono::steady_clock::now();
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
            patch_msg.type = static_cast<uint8_t>(n->getUserData().nodeType);
            patch_msg.type_name = nodeTypeName(n->getUserData().nodeType);
            patch_msg.obstacle_cause = static_cast<uint8_t>(n->getUserData().obstacleCause);
            patch_msg.obstacle_cause_name = obstacleCauseName(n->getUserData().obstacleCause);
            patch_msg.slope = n->getUserData().slope;
            patch_msg.slope_direction = n->getUserData().slopeDirectionAtan2;
            patch_msg.cost = n->getUserData().cost;
            for (const auto& allowed : n->getUserData().allowedOrientations){
                patch_msg.allowed_orientation_starts.push_back(allowed.getStart().getRad());
                patch_msg.allowed_orientation_widths.push_back(allowed.getWidth());
            }

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
                    // teal; the old dark magenta was hard to tell from the
                    // STEP_HEIGHT cause purple when cause-coloring is on
                    patch_msg.color.r = 0.0;
                    patch_msg.color.g = 0.55;
                    patch_msg.color.b = 0.55;
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
                    // blue-grey; plain 0.3 grey was indistinguishable from the
                    // UNMEASURED cause grey (0.35)
                    patch_msg.color.r = 0.25;
                    patch_msg.color.g = 0.32;
                    patch_msg.color.b = 0.45;
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
    RCLCPP_INFO_STREAM(this->get_logger(), "Timing: trav-map message build+publish ("
        << map_msg.patches.size() << " patches) took "
        << std::chrono::duration<double>(std::chrono::steady_clock::now() - t_publish).count()
        << " s.");
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
