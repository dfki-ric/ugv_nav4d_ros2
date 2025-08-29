#include "ugv_nav4d_ros2.hpp"
#include "util_functions.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h> // For removeNaNFromPointCloud

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <fstream>

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
    min_point.z() = get_parameter("dist_min_z").as_int(); 

    max_point.x() = get_parameter("dist_max_x").as_int();
    max_point.y() = get_parameter("dist_max_y").as_int(); 
    max_point.z() = get_parameter("dist_max_z").as_int(); 

    box_filter.setMin(min_point);  // Set minimum bound
    box_filter.setMax(max_point);  // Set maximum bound
    
    mls_min_x = get_parameter("dist_min_x").as_int();
    mls_min_y = get_parameter("dist_min_y").as_int();

    extend_trajectory = get_parameter("extend_trajectory").as_bool();    
    extension_distance = get_parameter("extension_distance").as_double();

    if (get_parameter("load_mls_from_file").as_bool()){
        const std::string mls_file_path = get_parameter("mls_file_path").as_string();
        const std::string mls_file_type = get_parameter("mls_file_type").as_string();

        if (mls_file_type == "ply"){        
            if (loadPlyAsMLS(mls_file_path)){
                got_map = true;
            }
        }
        else if (mls_file_type == "bin"){  
            if(loadMLSMapFromBin(mls_file_path)){
                got_map = true;
            }
        }
        else{
            throw std::runtime_error("Invalid MLS File Type: "+ mls_file_type);
        }
    }
    else{
        const double mls_res = get_parameter("grid_resolution").as_double();
        const double dist_max_x = get_parameter("dist_max_x").as_int();
        const double dist_max_y = get_parameter("dist_max_y").as_int();
        const double dist_min_x = get_parameter("dist_min_x").as_int();
        const double dist_min_y = get_parameter("dist_min_y").as_int();
        const double grid_size_x = (dist_max_x - dist_min_x)/mls_res;
        const double grid_size_y = (dist_max_y - dist_min_y)/mls_res;

        maps::grid::MLSConfig cfg;
        cfg.gapSize = get_parameter("mls_gap_size").as_double();
        const maps::grid::Vector2ui numCells(grid_size_x, grid_size_y);
        mls_map = maps::grid::MLSMapSloped(numCells, maps::grid::Vector2d(mls_res, mls_res), cfg);
        mls_map.translate(Eigen::Vector3d(mls_min_x, mls_min_y, 0));
    }

    combined_path_publisher = this->create_publisher<nav_msgs::msg::Path>("/ugv_nav4d_ros2/path", 10);
    labeled_path_publisher = this->create_publisher<ugv_nav4d_ros2::msg::LabeledPathArray>("/ugv_nav4d_ros2/labeled_path_segments", 10);
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
        std::bind(&PathPlannerNode::handle_save_map_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PathPlannerNode::handle_save_map_cancel, this, std::placeholders::_1),
        std::bind(&PathPlannerNode::handle_save_map_accepted, this, std::placeholders::_1)
    );

    // Map publisher trigger service
    map_publish_service = this->create_service<std_srvs::srv::Trigger>(
            "/ugv_nav4d_ros2/map_publish", std::bind(&PathPlannerNode::mapPublishCallback, this, std::placeholders::_1, std::placeholders::_2));

    sub_goal_pose = create_subscription<geometry_msgs::msg::PoseStamped>("/ugv_nav4d_ros2/goal_pose", 1, 
            bind(&PathPlannerNode::processGoalRequest, this, std::placeholders::_1));

    sub_start_pose = create_subscription<geometry_msgs::msg::PoseStamped>("/ugv_nav4d_ros2/start_pose", 1, 
            bind(&PathPlannerNode::readStartPose, this, std::placeholders::_1));

    parameter_callback_handle = this->add_on_set_parameters_callback(
        std::bind(&PathPlannerNode::parametersCallback, this, std::placeholders::_1));

    timer = this->create_wall_timer(
        std::chrono::milliseconds(1000),  // Timer period
        std::bind(&PathPlannerNode::parameterUpdateTimerCallback, this)  // Callback function
    );

    if (!get_parameter("load_mls_from_file").as_bool()){
        cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/ugv_nav4d_ros2/pointcloud", 10,
                std::bind(&PathPlannerNode::cloudCallback, this, std::placeholders::_1));
    }
}

void PathPlannerNode::parameterUpdateTimerCallback(){
    if (!parameters_to_update.empty())
    {
        this->set_parameters(parameters_to_update);
        parameters_to_update.clear();
        configurePlanner();
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
            RCLCPP_INFO(this->get_logger(), "Parameter '%s' changed.", param.get_name().c_str());
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
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not declared in the node.", param.get_name().c_str());
            result.successful = false;
            result.reason = "Parameter not found.";
        }
    }

    return result;
}


void PathPlannerNode::mapPublishCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received service request to publish maps.");
    publishMaps();
    response->success = true;
    response->message = "Published MLS and Traversability Map.";
    RCLCPP_INFO(this->get_logger(), "Published MLS and Traversability Map.");

}

void PathPlannerNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    latest_pointcloud = msg;
    got_map = generateMLS();

    if (got_map){
        if (!is_planning){

            if (!get_parameter("read_pose_from_topic").as_bool())
            {
                if (!read_pose_from_tf()){
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to read start pose of the robot from TF!");
                    return;
                }
            }

            if (!is_configured){
                configurePlanner();
            }

            RCLCPP_INFO(this->get_logger(), "Planner state: Got Map");
            mls_map_ptr = std::make_shared<traversability_generator3d::TraversabilityGenerator3d::MLGrid>(mls_map);
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
                RCLCPP_INFO(this->get_logger(), "Initial patch added to MLS.");
            }

            auto startPosition = ground2Mls.translation();
            traversability_generator_ptr->expandAll(startPosition);
            auto travMap = traversability_generator_ptr->getTraversabilityMap();
            planner_ptr->updateMap(travMap);
            RCLCPP_INFO(this->get_logger(), "Planner state: Ready");
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Map not loaded because planner is in state: Planning");
        }
    }
    else{
        RCLCPP_WARN_STREAM(this->get_logger(), "Unabled to load map from incoming cloud!");
    }
}

bool PathPlannerNode::read_pose_from_tf(){
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
    if (!is_configured){
        configurePlanner();
    }

    if (!got_map){
        RCLCPP_WARN_STREAM(this->get_logger(), "Unabled to process goal request because planner is in state NO_MAP!");
        return;
    }

    if (is_planning){
        RCLCPP_WARN_STREAM(this->get_logger(), "Unabled to process goal request because planner is in state PLANNING!");
        return;
    }

    if (!get_parameter("read_pose_from_topic").as_bool())
    {
        if (!read_pose_from_tf()){
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to read start pose of the robot from TF!");
            return;
        }
    }

    start_pose_rbs.position = Eigen::Vector3d(start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z);
    start_pose_rbs.orientation = Eigen::Quaterniond(start_pose.pose.orientation.w, start_pose.pose.orientation.x, start_pose.pose.orientation.y, start_pose.pose.orientation.z);

    goal_pose_rbs.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    goal_pose_rbs.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    
    plan();
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

            mls_min_x = min.x;
            mls_min_y = min.y;

            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            box_filter.setInputCloud(cloud);
            box_filter.filter(*cloud_filtered);
    
            const double mls_res = get_parameter("grid_resolution").as_double();
            const double size_x = max.x - min.x;
            const double size_y = max.y - min.y;
            
            maps::grid::MLSConfig cfg;
            cfg.gapSize = get_parameter("mls_gap_size").as_double();
            const maps::grid::Vector2ui numCells(size_x / mls_res + 1, size_y / mls_res + 1);
            mls_map = maps::grid::MLSMapSloped(numCells, maps::grid::Vector2d(mls_res, mls_res), cfg);
            mls_map.translate(Eigen::Vector3d(min.x, min.y, 0));
            mls_map.mergePointCloud(*cloud_filtered, base::Transform3d::Identity());
        }
        return true;
    }
    RCLCPP_ERROR_STREAM(this->get_logger(), "Unabled to load mls. Unknown format!");
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

    mls_map.mergePointCloud(*cloud_filtered, cloud2MLS);
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
        std::cerr << "Error opening file for writing: " << fileToUse << std::endl;
        return false;
    }

    // Create a binary archive
    boost::archive::binary_oarchive archive(binFile);
    archive << mls_map;

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
        RCLCPP_WARN_STREAM(this->get_logger(), "Failed to open file: " << filename);
        return false;
    }

    try {
        // Load the file contents into the stream and deserialize
        boost::archive::binary_iarchive ia(file);
        ia >> mls_map;  // Deserialize into mls_map

        RCLCPP_INFO_STREAM(this->get_logger(), "Loaded MLS Map from " << filename);
        return true;
    } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Error loading MLS Map: " << e.what());
        return false;
    }
}

rclcpp_action::GoalResponse PathPlannerNode::handle_save_map_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SaveMLSMap::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received save map request");
    (void)uuid;  // Suppress unused variable warning
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;  // Accept the goal
}

rclcpp_action::CancelResponse PathPlannerNode::handle_save_map_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SaveMLSMap>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Canceling save map request");
    return rclcpp_action::CancelResponse::ACCEPT;  // Accept the cancel request
}

void PathPlannerNode::handle_save_map_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SaveMLSMap>> goal_handle)
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

void PathPlannerNode::plan(){

    std::vector<trajectory_follower::SubTrajectory> trajectory2D, trajectory3D;
    base::Time time;
    time.microseconds = get_parameter("planningTime").as_int();

    bool dumpOnError = get_parameter("dumpOnError").as_bool();
    bool dumpOnSuccess = get_parameter("dumpOnSuccess").as_bool();

    RCLCPP_INFO_STREAM(this->get_logger(), "Planner state: Planning");
    is_planning = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "Start is  " << start_pose_rbs.position.transpose());
    RCLCPP_INFO_STREAM(this->get_logger(), "Goal is  " << goal_pose_rbs.position.transpose());
    ugv_nav4d::Planner::PLANNING_RESULT res = planner_ptr->plan(time, start_pose_rbs, goal_pose_rbs, trajectory2D, trajectory3D, dumpOnError, dumpOnSuccess);
    is_planning = false;

    switch(res)
    {
        case ugv_nav4d::Planner::FOUND_SOLUTION:
            RCLCPP_INFO_STREAM(this->get_logger(), "FOUND_SOLUTION");
            break;
        case ugv_nav4d::Planner::GOAL_INVALID:
            RCLCPP_INFO_STREAM(this->get_logger(), "GOAL_INVALID");
            break;
        case ugv_nav4d::Planner::START_INVALID:
            RCLCPP_INFO_STREAM(this->get_logger(), "START_INVALID");
            break;
        case ugv_nav4d::Planner::INTERNAL_ERROR:
            RCLCPP_INFO_STREAM(this->get_logger(), "INTERNAL_ERROR");
            break;
        case ugv_nav4d::Planner::NO_SOLUTION:
            RCLCPP_INFO_STREAM(this->get_logger(), "NO_SOLUTION");
            break;
        case ugv_nav4d::Planner::NO_MAP:
            RCLCPP_INFO_STREAM(this->get_logger(), "NO_MAP");
            break;
    }
    
    ugv_nav4d_ros2::msg::LabeledPathArray labeled_path_message;
    auto now = this->get_clock()->now();

    nav_msgs::msg::Path path;
    path.header.frame_id = get_parameter("world_frame").as_string();

    nav_msgs::msg::Path path_segment;
    std::string label_last;
    std::string label;
    bool first_segment = true;

    if (res == ugv_nav4d::Planner::FOUND_SOLUTION) {
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
                throw std::runtime_error("Invalid DriveMode: " + std::to_string(trajectory.driveMode));
            }

            // If this is a new segment (first or label changed), start a new path_segment
            if (first_segment || label != label_last) {
                if (!first_segment && !path_segment.poses.empty()) {
                    labeled_path_message.paths.push_back(path_segment);
                    labeled_path_message.labels.push_back(label_last);
                }

                path_segment = nav_msgs::msg::Path();
                path_segment.header.frame_id = get_parameter("world_frame").as_string();
                label_last = label;
                first_segment = false;
            }

            // Sample spline and add poses to current segment
            const double stepDist = get_parameter("spline_resolution_distance").as_double();
            std::vector<double> parameters;
            const std::vector<base::geometry::Spline3::vector_t> points = trajectory.posSpline.sample(stepDist, &parameters);
            assert(parameters.size() == points.size());
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
                tempPoint.header.frame_id = get_parameter("world_frame").as_string();

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

        combined_path_publisher->publish(path);
        labeled_path_publisher->publish(labeled_path_message);
    }

}

void PathPlannerNode::declareParameters(){

    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.read_only = true;

    declare_parameter("spline_resolution_distance", 0.1, param_desc);
    declare_parameter("read_pose_from_topic", false, param_desc);
    declare_parameter("load_mls_from_file", false, param_desc);
    declare_parameter("mls_file_type", "ply", param_desc);
    declare_parameter("mls_file_path", "default_value", param_desc);
    declare_parameter("robot_frame", "robot", param_desc);
    declare_parameter("world_frame", "map", param_desc);
    declare_parameter("grid_resolution", 0.3, param_desc);
    declare_parameter("mls_gap_size", 0.1, param_desc);
    declare_parameter("dist_max_x", 50, param_desc);
    declare_parameter("dist_min_x", -50, param_desc);
    declare_parameter("dist_max_y", 50, param_desc);
    declare_parameter("dist_min_y", -50, param_desc);
    declare_parameter("dist_max_z", 50, param_desc);
    declare_parameter("dist_min_z", -50, param_desc);
    declare_parameter("initialPatchRadius", 3.0, param_desc);

    declare_parameter("dumpOnError", false);
    declare_parameter("dumpOnSuccess", false);

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

    declare_parameter("epsilonSteps", 2);
    declare_parameter("initialEpsilon", 64);
    declare_parameter("numThreads", 8);
    declare_parameter("planningTime", 5000000); // microseconds

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
    declare_parameter("enableInclineLimitting", true);
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
    //declare_parameter("slopeMetric", :NONE);
    declare_parameter("slopeMetricScale", 1.0);    

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
    mobility_config.spline_sampling_resolution            = get_parameter("spline_sampling_resolution").as_double();
    mobility_config.remove_goal_offset                    = get_parameter("remove_goal_offset").as_bool();

    traversability_config.gridResolution            = get_parameter("grid_resolution").as_double();
    traversability_config.maxSlope                  = get_parameter("maxSlope").as_double();
    traversability_config.maxStepHeight             = get_parameter("maxStepHeight").as_double();
    traversability_config.robotSizeX                = get_parameter("robotSizeX").as_double();
    traversability_config.robotSizeY                = get_parameter("robotSizeY").as_double();
    traversability_config.robotHeight               = get_parameter("robotHeight").as_double();
    traversability_config.slopeMetricScale          = get_parameter("slopeMetricScale").as_double();
    //TODO: How can an enum be used in the parameter config? (Probably Int to Enum cast works)
    traversability_config.slopeMetric = traversability_generator3d::SlopeMetric::NONE;
    traversability_config.inclineLimittingMinSlope  = get_parameter("inclineLimittingMinSlope").as_double(); 
    traversability_config.inclineLimittingLimit     = get_parameter("inclineLimittingLimit").as_double();
    traversability_config.costFunctionDist          = get_parameter("costFunctionDist").as_double();
    traversability_config.distToGround              = get_parameter("distToGround").as_double();
    traversability_config.minTraversablePercentage  = get_parameter("minTraversablePercentage").as_double();
    traversability_config.allowForwardDownhill      = get_parameter("allowForwardDownhill").as_bool();

    planner_config.epsilonSteps                     = get_parameter("epsilonSteps").as_int();
    planner_config.initialEpsilon                   = get_parameter("initialEpsilon").as_int();
    planner_config.numThreads                       = get_parameter("numThreads").as_int(); 
}

void PathPlannerNode::configurePlanner(){
    updateParameters();
    planner_ptr.reset(new ugv_nav4d::Planner(spline_primitive_config, traversability_config, mobility_config, planner_config));
    traversability_generator_ptr.reset(new traversability_generator3d::TraversabilityGenerator3d(traversability_config));
    is_configured = true;

    if(got_map){//If map is already available then load the last known map.
        if (!get_parameter("read_pose_from_topic").as_bool())
        {
            if (!read_pose_from_tf()){
                RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to read start pose of the robot from TF!");
                return;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Planner state: Loading last known map");
        mls_map_ptr = std::make_shared<traversability_generator3d::TraversabilityGenerator3d::MLGrid>(mls_map);
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
        auto travMap = traversability_generator_ptr->getTraversabilityMap();
        planner_ptr->updateMap(travMap);
        RCLCPP_INFO(this->get_logger(), "Planner state: Ready");
    }
}

void PathPlannerNode::publishMaps(){
    publishMLSMap();
    publishTravMap();
}

bool PathPlannerNode::publishMLSMap(){
    ugv_nav4d_ros2::msg::MLSMap map_msg;    
    map_msg.width = 1;
    map_msg.height = 1;
    map_msg.depth = 1;
    map_msg.resolution = get_parameter("grid_resolution").as_double();
    map_msg.header.frame_id = get_parameter("world_frame").as_string();

    mls_map = *mls_map_ptr;
    maps::grid::Vector2ui num_cell = mls_map.getNumCells();
    typedef maps::grid::MLSMap<maps::grid::MLSConfig::SLOPE>::CellType Cell;

    for (size_t x = 0; x < num_cell.x(); x++)
    {
        for (size_t y = 0; y < num_cell.y(); y++)
        {
            const Cell &list = mls_map.at(x, y);
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
                pos = (maps::grid::Index(x, y).cast<double>() + maps::grid::Vector2d(0.5, 0.5)).array() * mls_map.getResolution().array();
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
        RCLCPP_WARN(this->get_logger(), "Empty MLS Map!");
        return false;
    }

    mls_map_publisher->publish(map_msg);
    return true;
}

void PathPlannerNode::publishTravMap(){
    const auto& trav_map_3d = traversability_generator_ptr->getTraversabilityMap(); 
    ugv_nav4d_ros2::msg::TravMap msg;
    msg.width = 1;
    msg.height = 1;
    msg.depth = 1;
    msg.resolution = get_parameter("grid_resolution").as_double();
    msg.header.frame_id = get_parameter("world_frame").as_string();

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
            }
            msg.patches.push_back(patch_msg);

        }
    }
    trav_map_publisher->publish(msg);
}
}
