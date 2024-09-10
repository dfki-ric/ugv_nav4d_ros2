#include "ugv_nav4d_ros2.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <fstream>

using namespace rclcpp;

namespace ugv_nav4d_ros2 {

PathPlannerNode::PathPlannerNode()
    : Node("ugv_nav4d_ros2")
{
    declareParameters();

    // controller feedback (via TF)
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    initialPatchAdded = false;

    timer_pose_samples = create_wall_timer(std::chrono::duration<double>(0.01), std::bind(&PathPlannerNode::pose_samples_callback, this));
    timer_map_publish = create_wall_timer(std::chrono::seconds(5), std::bind(&PathPlannerNode::map_publish_callback, this));
    sub_goal_pose = create_subscription<geometry_msgs::msg::PoseStamped>("/ugv_nav4d_ros2/goal_pose", 1, bind(&PathPlannerNode::process_goal_request, this, std::placeholders::_1));

    // Create a parameter subscriber that can be used to monitor parameter changes
    // (for this node's parameters as well as other nodes' parameters)
    param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);

    cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ugv_nav4d_ros2/pointcloud", 10,
      std::bind(&PathPlannerNode::cloud_callback, this, std::placeholders::_1));

    // Set a callback for this node's parameter updates
    auto cb = [this](const rclcpp::Parameter & p) {
        RCLCPP_INFO(
          this->get_logger(), "cb: Received an update to parameter \"%s\" of type %s: \"%ld\"",
          p.get_name().c_str(),
          p.get_type_name().c_str(),
          p.as_int());
      };

    cb_handle = param_subscriber->add_parameter_callback("param_subscriber", cb);
    path_publisher = this->create_publisher<nav_msgs::msg::Path>("/ugv_nav4d_ros2/path", 10);
    grid_map_publisher = this->create_publisher<nav_msgs::msg::GridCells>("/ugv_nav4d_ros2/grid_map", 10);
    trav_map_publisher = this->create_publisher<ugv_nav4d_ros2::msg::TravMap>("/ugv_nav4d_ros2/trav_map", 10);
    mls_map_publisher = this->create_publisher<ugv_nav4d_ros2::msg::MLSMap>("/ugv_nav4d_ros2/mls_map", 10);
    cloud_map_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ugv_nav4d_ros2/cloud_map", 10);

    configurePlanner();

    if (get_parameter("map_ply_path").as_string() != "default_value" && get_parameter("load_map_from_topic").as_bool() == false){
        //Load map from file only when user explicity sets the parameter
        RCLCPP_INFO_STREAM(this->get_logger(), "Loading map from file: " + get_parameter("map_ply_path").as_string());
        loadMls(get_parameter("map_ply_path").as_string());
    }

    const double mls_res = get_parameter("grid_resolution").as_double();

    const double grid_max_x = get_parameter("grid_max_x").as_int();
    const double grid_max_y = get_parameter("grid_max_y").as_int();

    const double grid_min_x = get_parameter("grid_min_x").as_int();
    const double grid_min_y = get_parameter("grid_min_y").as_int();

    const double grid_size_x = (grid_max_x - grid_min_x)/mls_res;
    const double grid_size_y = (grid_max_y - grid_min_y)/mls_res;
    
    maps::grid::MLSConfig cfg;
    cfg.gapSize = get_parameter("grid_resolution").as_double();
    const maps::grid::Vector2ui numCells(grid_size_x, grid_size_y);
    mlsMap = maps::grid::MLSMapSloped(numCells, maps::grid::Vector2d(mls_res, mls_res), cfg);
    mlsMap.translate(Eigen::Vector3d(grid_min_x, grid_min_y, 0));
}

void PathPlannerNode::map_publish_callback()
{
    if (publishMLSMap()){
        RCLCPP_INFO(this->get_logger(), "Published MLS map.");  
    }
}

void PathPlannerNode::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    latest_pointcloud = msg;

    bool load_map_from_topic = get_parameter("load_map_from_topic").as_bool();
    if (load_map_from_topic){
        if (pose_samples_callback()){
            generateMls();
        }
        else{
            RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: Failed to read robot pose! No map generated.");
        }
            
    }
}

bool PathPlannerNode::pose_samples_callback(){

    //printConfigs();
    std::string robot_frame = get_parameter("robot_frame").as_string();
    std::string world_frame = get_parameter("world_frame").as_string();

    try{
        geometry_msgs::msg::TransformStamped t = tf_buffer->lookupTransform(world_frame, robot_frame, tf2::TimePointZero);
        pose_samples.pose.orientation = t.transform.rotation;
        pose_samples.pose.position.x =  t.transform.translation.x;
        pose_samples.pose.position.y =  t.transform.translation.y;
        pose_samples.pose.position.z =  t.transform.translation.z;
    }
    catch(const tf2::TransformException & ex){
        return false;
    }
    return true;    
}

void PathPlannerNode::process_goal_request(const geometry_msgs::msg::PoseStamped::SharedPtr msg){

    start_pose_rbs.position = Eigen::Vector3d(pose_samples.pose.position.x,
                                              pose_samples.pose.position.y,
                                              pose_samples.pose.position.z);

    start_pose_rbs.orientation = Eigen::Quaterniond(pose_samples.pose.orientation.w,
                                                    pose_samples.pose.orientation.x,
                                                    pose_samples.pose.orientation.y,
                                                    pose_samples.pose.orientation.z);

    goal_pose_rbs.position = Eigen::Vector3d(msg->pose.position.x,
                                             msg->pose.position.y,
                                             msg->pose.position.z);

    goal_pose_rbs.orientation = Eigen::Quaterniond(msg->pose.orientation.w,
                                                   msg->pose.orientation.x,
                                                   msg->pose.orientation.y,
                                                   msg->pose.orientation.z);

    RCLCPP_INFO_STREAM(this->get_logger(), "Start Position: " << start_pose_rbs.position.transpose());
    RCLCPP_INFO_STREAM(this->get_logger(), "Goal Position: "  << goal_pose_rbs.position.transpose());

    //The z = 00 is a hack for now
    start_pose_rbs.position = Eigen::Vector3d(pose_samples.pose.position.x,
                                              pose_samples.pose.position.y,
                                              0.0);

    plan();
}

void PathPlannerNode::printPlannerConfig(){

    //TODO: Add other config params to printout
    RCLCPP_INFO_STREAM(this->get_logger(), "SplinePrimitivesConfig");
    RCLCPP_INFO_STREAM(this->get_logger(), "gridSize: " << splinePrimitiveConfig.gridSize);
    RCLCPP_INFO_STREAM(this->get_logger(), "numAngles: " << splinePrimitiveConfig.numAngles);
    RCLCPP_INFO_STREAM(this->get_logger(), "numEndAngles: " << splinePrimitiveConfig.numEndAngles);
    RCLCPP_INFO_STREAM(this->get_logger(), "destinationCircleRadius: " << splinePrimitiveConfig.destinationCircleRadius);
    RCLCPP_INFO_STREAM(this->get_logger(), "cellSkipFactor: " << splinePrimitiveConfig.cellSkipFactor);
    RCLCPP_INFO_STREAM(this->get_logger(), "splineOrder: " << splinePrimitiveConfig.splineOrder);
    RCLCPP_INFO_STREAM(this->get_logger(), "generateForwardMotions: " << splinePrimitiveConfig.generateForwardMotions);
    RCLCPP_INFO_STREAM(this->get_logger(), "generateBackwardMotions: " << splinePrimitiveConfig.generateBackwardMotions);
    RCLCPP_INFO_STREAM(this->get_logger(), "generateLateralMotions: " << splinePrimitiveConfig.generateLateralMotions);
    RCLCPP_INFO_STREAM(this->get_logger(), "generatePointTurnMotions: " << splinePrimitiveConfig.generatePointTurnMotions);
}

bool PathPlannerNode::loadMls(const std::string& path){

    std::ifstream fileIn(path);       

    if(path.find(".ply") != std::string::npos)
    {
        cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

        RCLCPP_INFO_STREAM(this->get_logger(), "Loading PLY: " << get_parameter("map_ply_path").as_string());        
        pcl::PLYReader plyReader;
        if(plyReader.read(path, *cloud) >= 0)
        {
            pcl::PointXYZ min, max; 
            pcl::getMinMax3D (*cloud, min, max); 
        
            //transform point cloud to zero (instead we could also use MlsMap::translate later but that seems to be broken?)
            Eigen::Affine3f pclTf = Eigen::Affine3f::Identity();
            pclTf.translation() << -min.x, -min.y, -min.z;
            pcl::transformPointCloud (*cloud, *cloud, pclTf);
            
            //pcl::getMinMax3D (*cloud, min, max); 

            const double mls_res = get_parameter("grid_resolution").as_double();
            const double size_x = max.x - min.x;
            const double size_y = max.y - min.y;
            
            maps::grid::MLSConfig cfg;
            cfg.gapSize = get_parameter("grid_resolution").as_double();
            const maps::grid::Vector2ui numCells(size_x / mls_res + 1, size_y / mls_res + 1);
            mlsMap = maps::grid::MLSMapSloped(numCells, maps::grid::Vector2d(mls_res, mls_res), cfg);
            mlsMap.mergePointCloud(*cloud, base::Transform3d::Identity());
            RCLCPP_INFO_STREAM(this->get_logger(), "MLS Resolution: " << get_parameter("grid_resolution").as_double());
            RCLCPP_INFO_STREAM(this->get_logger(), "NUM CELLS: " << numCells);
            RCLCPP_INFO_STREAM(this->get_logger(), "Cloud Points: " << cloud->size());
            RCLCPP_INFO_STREAM(this->get_logger(), "Generated MLS Map. Loading the Map into Planner...");
            planner->updateMap(mlsMap);
            RCLCPP_INFO_STREAM(this->get_logger(), "Loaded Map into Planner");
            //publishMLSMap();
        }
        return true;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Unabled to load mls. Unknown format");
    return false;
}

bool PathPlannerNode::generateMls(){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*latest_pointcloud, *cloud);

    std::string world_frame = get_parameter("world_frame").as_string();
    base::Transform3d cloud2MLS;

    try{
        geometry_msgs::msg::TransformStamped t = tf_buffer->lookupTransform(world_frame, "husky/base_link/front_laser", tf2::TimePointZero);
        cloud2MLS.translation() << t.transform.translation.x, t.transform.translation.y, t.transform.translation.z;
        Eigen::Quaterniond quat(t.transform.rotation.w, 
                                t.transform.rotation.x, 
                                t.transform.rotation.y, 
                                t.transform.rotation.z);
        cloud2MLS.linear() = quat.toRotationMatrix();
    }
    catch(const tf2::TransformException & ex){
        return false;
    }

    mlsMap.mergePointCloud(*cloud, cloud2MLS);
    RCLCPP_INFO_STREAM(this->get_logger(), "MLS Resolution: " << get_parameter("grid_resolution").as_double());
    RCLCPP_INFO_STREAM(this->get_logger(), "Cloud Points: " << cloud->size());
    RCLCPP_INFO_STREAM(this->get_logger(), "Generated MLS Map. Loading the Map into Planner...");
    RCLCPP_INFO_STREAM(this->get_logger(), "MLS Cells: " << mlsMap.getNumCells().transpose());

    planner->updateMap(mlsMap);
    RCLCPP_INFO_STREAM(this->get_logger(), "Loaded Map into Planner");
    if(!initialPatchAdded)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Setting Initial Patch of size: " << get_parameter("initialPatchRadius").as_double());
        planner->setInitialPatch(start_pose_rbs.getTransform(), get_parameter("initialPatchRadius").as_double());
        initialPatchAdded = true;
    }

    //Eigen::Affine3d pclTf = Eigen::Affine3d::Identity();
    //pclTf.translation() = cloud2MLS.translation();
    //pclTf.linear() = cloud2MLS.linear();
    //pcl::transformPointCloud (*cloud, *cloud, pclTf);

    //sensor_msgs::msg::PointCloud2::SharedPtr cloud_ros = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // Convert PCL PointCloud to ROS PointCloud2 message
    //pcl::toROSMsg(*cloud, *cloud_ros);
    //cloud_ros->header.frame_id = get_parameter("world_frame").as_string();
    //cloud_ros->header.stamp = this->now();
    //cloud_map_publisher->publish(*cloud_ros);
    //publishMLSMap();

    return true;
}


void PathPlannerNode::plan(){

    std::vector<trajectory_follower::SubTrajectory> trajectory2D, trajectory3D;
    base::Time time;
    time.microseconds = get_parameter("planningTime").as_int();

    RCLCPP_INFO_STREAM(this->get_logger(), "Planning...");
    ugv_nav4d::Planner::PLANNING_RESULT res = planner->plan(time, start_pose_rbs, goal_pose_rbs, trajectory2D, trajectory3D, false, false);

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

    nav_msgs::msg::Path path_message;

    //Publish path if a solution is found
    if (res == ugv_nav4d::Planner::FOUND_SOLUTION){
        for (auto& trajectory : trajectory3D){

            //sample spline:
            const double stepDist = 0.01;
            std::vector<double> parameters;
            //NOTE we dont need the points, but there is no sample() api that returns parameters only
            const std::vector<base::geometry::Spline3::vector_t> points = trajectory.posSpline.sample(stepDist, &parameters);
            assert(parameters.size() == points.size());
            for(size_t i = 0; i < parameters.size(); ++i)
            {
                const double param = parameters[i];
                base::Vector3d point, tangent;
                std::tie(point,tangent) = trajectory.posSpline.getPointAndTangent(param);
                //const base::Orientation orientation(std::atan2(tangent.y(), tangent.x()));

                //point needs to be offset to the middle of the grid,
                //as all path computation also starts in the middle
                //if not we would get a wrong diff
                point += base::Vector3d(traversabilityConfig.gridResolution /2.0, traversabilityConfig.gridResolution /2.0,0);

                // Fill current path point to a temporary variable.
                //TODO: Set the z path to the patch height to get a 3D Trajectory
                geometry_msgs::msg::PoseStamped tempPoint;
                tempPoint.pose.position.x= point.x();
                tempPoint.pose.position.y= point.y();
                tempPoint.pose.position.z= get_parameter("distToGround").as_double();
                // Add points to path.
                path_message.header.frame_id = get_parameter("world_frame").as_string();
                path_message.poses.push_back(tempPoint);
            }
        }
        path_publisher->publish(path_message);
        RCLCPP_INFO_STREAM(this->get_logger(), "Published Path...");
        publishTravMap();
    }
}

void PathPlannerNode::declareParameters(){

    declare_parameter("load_map_from_topic", true);
    declare_parameter("map_ply_path", "default_value");

    declare_parameter("robot_frame", "robot");
    declare_parameter("world_frame", "map");
    declare_parameter("grid_resolution", 0.3);

    declare_parameter("grid_max_x", 50);
    declare_parameter("grid_min_x", -50);
    declare_parameter("grid_max_y", 50);
    declare_parameter("grid_min_y", -50);

    declare_parameter("dumpOnError", 0);
    declare_parameter("dumpOnSuccess", 0);
    declare_parameter("initialPatchRadius", 3.0);

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
    declare_parameter("searchRadius", 0.0);
    declare_parameter("translationSpeed", 1.0);

    declare_parameter("epsilonSteps", 2);
    declare_parameter("initialEpsilon", 64);
    declare_parameter("numThreads", 8);
    declare_parameter("planningTime", 50000000); // microseconds

    declare_parameter("cellSkipFactor", 3);
    declare_parameter("destinationCircleRadius", 10);
    declare_parameter("generateBackwardMotions", true);
    declare_parameter("generateForwardMotions", true);
    declare_parameter("generateLateralMotions", false);
    declare_parameter("generatePointTurnMotions", true);
    declare_parameter("numAngles", 42);
    declare_parameter("numEndAngles", 21);
    declare_parameter("splineOrder", 4);

    declare_parameter("allowForwardDownhill", true);
    declare_parameter("costFunctionDist", 0.0);
    declare_parameter("distToGround", 0.0);
    declare_parameter("enableInclineLimitting", true);
    declare_parameter("inclineLimittingLimit", 0.1);
    declare_parameter("inclineLimittingMinSlope", 0.2);
    declare_parameter("initialPatchVariance", 0.0001);
    declare_parameter("maxSlope", 0.45);
    declare_parameter("maxStepHeight", 0.2);
    declare_parameter("minTraversablePercentage", 0.4);
    declare_parameter("robotHeight", 1.7);
    declare_parameter("robotSizeX", 0.80);
    declare_parameter("robotSizeY", 0.80);
    //declare_parameter("slopeMetric", :NONE);
    declare_parameter("slopeMetricScale", 1.0);    
}

void PathPlannerNode::updateParameters(){

    splinePrimitiveConfig.gridSize                 = get_parameter("grid_resolution").as_double();
    splinePrimitiveConfig.numAngles                = get_parameter("numAngles").as_int();
    splinePrimitiveConfig.numEndAngles             = get_parameter("numEndAngles").as_int();
    splinePrimitiveConfig.destinationCircleRadius  = get_parameter("destinationCircleRadius").as_int();
    splinePrimitiveConfig.cellSkipFactor           = get_parameter("cellSkipFactor").as_int();
    splinePrimitiveConfig.generateForwardMotions   = get_parameter("generateForwardMotions").as_bool();
    splinePrimitiveConfig.generatePointTurnMotions = get_parameter("generatePointTurnMotions").as_bool();
    splinePrimitiveConfig.generateLateralMotions   = get_parameter("generateLateralMotions").as_bool();
    splinePrimitiveConfig.generateBackwardMotions  = get_parameter("generateBackwardMotions").as_bool();
    splinePrimitiveConfig.splineOrder              = get_parameter("splineOrder").as_int();
    
    mobility.translationSpeed                      = get_parameter("translationSpeed").as_double();
    mobility.rotationSpeed                         = get_parameter("rotationSpeed").as_double();
    mobility.minTurningRadius                      = get_parameter("minTurningRadius").as_double();
    mobility.searchRadius                          = get_parameter("searchRadius").as_double();
    mobility.multiplierForward                     = get_parameter("multiplierForward").as_double();
    mobility.multiplierBackward                    = get_parameter("multiplierBackward").as_double();
    mobility.multiplierLateral                     = get_parameter("multiplierLateral").as_double();
    mobility.multiplierBackwardTurn                = get_parameter("multiplierBackwardTurn").as_double();
    mobility.multiplierForwardTurn                 = get_parameter("multiplierForwardTurn").as_double();
    mobility.multiplierPointTurn                   = get_parameter("multiplierPointTurn").as_double();
     
    traversabilityConfig.gridResolution            = get_parameter("grid_resolution").as_double();
    traversabilityConfig.maxSlope                  = get_parameter("maxSlope").as_double();
    traversabilityConfig.maxStepHeight             = get_parameter("maxStepHeight").as_double();
    traversabilityConfig.robotSizeX                = get_parameter("robotSizeX").as_double();
    traversabilityConfig.robotSizeY                = get_parameter("robotSizeY").as_double();
    traversabilityConfig.robotHeight               = get_parameter("robotHeight").as_double();
    traversabilityConfig.slopeMetricScale          = get_parameter("slopeMetricScale").as_double();
    //TODO: How can an enum be used in the parameter config? (Probably Int to Enum cast works)
    traversabilityConfig.slopeMetric = traversability_generator3d::SlopeMetric::NONE;
    traversabilityConfig.inclineLimittingMinSlope  = get_parameter("inclineLimittingMinSlope").as_double(); 
    traversabilityConfig.inclineLimittingLimit     = get_parameter("inclineLimittingLimit").as_double();
    traversabilityConfig.costFunctionDist          = get_parameter("costFunctionDist").as_double();
    traversabilityConfig.distToGround              = get_parameter("distToGround").as_double();
    traversabilityConfig.minTraversablePercentage  = get_parameter("minTraversablePercentage").as_double();
    traversabilityConfig.allowForwardDownhill      = get_parameter("allowForwardDownhill").as_bool();

    plannerConfig.epsilonSteps                     = get_parameter("epsilonSteps").as_int();
    plannerConfig.initialEpsilon                   = get_parameter("initialEpsilon").as_int();
    plannerConfig.numThreads                       = get_parameter("numThreads").as_int(); 
}

void PathPlannerNode::configurePlanner(){
    updateParameters();
    planner.reset(new ugv_nav4d::Planner(splinePrimitiveConfig, traversabilityConfig, mobility, plannerConfig));
}

bool PathPlannerNode::publishMLSMap(){

    ugv_nav4d_ros2::msg::MLSMap map_msg;    
    map_msg.width = 1;
    map_msg.height = 1;
    map_msg.depth = 1;
    map_msg.resolution = get_parameter("grid_resolution").as_double();
    map_msg.header.frame_id = get_parameter("world_frame").as_string();

    int minMeasurements = 1;
    maps::grid::Vector2ui num_cell = mlsMap.getNumCells();
    typedef maps::grid::MLSMap<maps::grid::MLSConfig::SLOPE>::CellType Cell;

    double grid_min_x = get_parameter("grid_min_x").as_int();
    double grid_min_y = get_parameter("grid_min_y").as_int();


    for (size_t x = 0; x < num_cell.x(); x++)
    {
        for (size_t y = 0; y < num_cell.y(); y++)
        {
            const Cell &list = mlsMap.at(x, y);
            for (Cell::const_iterator it = list.begin(); it != list.end(); it++)
            {
                const maps::grid::SurfacePatch<maps::grid::MLSConfig::SLOPE>& p = *it;  
                if(p.getNumberOfMeasurements() < minMeasurements){
                    //TODO: What does this do?
                    std::cout << "Too few measurements!" << std::endl;
                    continue;
                }

                float minZ, maxZ;
                p.getRange(minZ, maxZ);
                minZ -= 5e-4f;
                maxZ += 5e-4f;
                Eigen::Vector3f normal = p.getNormal();
                if(normal.z() < 0)
                    normal *= -1.0;

                if(normal.allFinite())
                {
                    Eigen::Hyperplane<float, 3> plane = Eigen::Hyperplane<float, 3>(normal, p.getCenter());
                    ugv_nav4d_ros2::msg::MLSPatch patch_msg;

                    patch_msg.color.r = 0;
                    patch_msg.color.g = 1;
                    patch_msg.color.b = 0;
                    patch_msg.color.a = 1;
                    maps::grid::Vector2d pos(0.00, 0.00);
                    // Calculate the position of the cell center.
                    pos = (maps::grid::Index(x, y).cast<double>() + maps::grid::Vector2d(0.5, 0.5)).array() * mlsMap.getResolution().array();
                    patch_msg.a = plane.normal()(0);
                    patch_msg.b = plane.normal()(1);
                    patch_msg.c = plane.normal()(2);
                    patch_msg.d = -plane.offset();                       
                    patch_msg.position.x = pos.x() + grid_min_x;
                    patch_msg.position.y = pos.y() + grid_min_y;
                    patch_msg.position.z = p.getCenter().z();
                    map_msg.patches.push_back(patch_msg);
                }
                else
                {
                    //TODO
                    //float height = (maxZ - minZ) + 1e-3f;
                    //geode.drawBox(maxZ, height, osg::Vec3(0.f,0.f,1.f));
                }
            }
        }
    }

    if (map_msg.patches.size() > 0){
        mls_map_publisher->publish(map_msg);
        return true;
    }
    else{
        return false;
    }
}

void PathPlannerNode::publishTravMap(){

    RCLCPP_INFO_STREAM(this->get_logger(), "Getting Traversability Map");
    const auto& trav_map_3d = planner->getTraversabilityMap(); 

    nav_msgs::msg::GridCells grid_map;
    grid_map.header.frame_id = get_parameter("world_frame").as_string();
    grid_map.cell_height = trav_map_3d.getResolution().x();
    grid_map.cell_width  = trav_map_3d.getResolution().y();

    ugv_nav4d_ros2::msg::TravMap msg;
    msg.width = 1;
    msg.height = 1;
    msg.depth = 1;
    msg.resolution = get_parameter("grid_resolution").as_double();
    msg.header.frame_id = get_parameter("world_frame").as_string();

    RCLCPP_INFO_STREAM(this->get_logger(), "Reading Traversability Map");
    for(const maps::grid::LevelList<traversability_generator3d::TravGenNode *> &l : trav_map_3d)
    {
        for(const traversability_generator3d::TravGenNode *n : l)
        {
            ugv_nav4d_ros2::msg::TravPatch patch_msg;
            const Eigen::Vector3d& position = n->getVec3(trav_map_3d.getResolution().x());
            geometry_msgs::msg::Point cell_center;
            cell_center.x = position.x();
            cell_center.y = position.y();
            cell_center.z = position.z();
            grid_map.cells.push_back(cell_center);

            patch_msg.a = n->getUserData().plane.normal()(0);
            patch_msg.b = n->getUserData().plane.normal()(1);
            patch_msg.c = n->getUserData().plane.normal()(2);
            patch_msg.d = -n->getUserData().plane.offset();
            patch_msg.position.x = position.x();
            patch_msg.position.y = position.y();
            patch_msg.position.z = position.z();

            switch((n->getType())){
                case maps::grid::TraversabilityNodeBase::TRAVERSABLE:
                    patch_msg.color.r = 0;
                    patch_msg.color.g = 1;
                    patch_msg.color.b = 0;
                    patch_msg.color.a = 1;
                    break;                    
                case maps::grid::TraversabilityNodeBase::OBSTACLE:
                    patch_msg.color.r = 1;
                    patch_msg.color.g = 0;
                    patch_msg.color.b = 0;
                    patch_msg.color.a = 1;
                    break;    
                case maps::grid::TraversabilityNodeBase::FRONTIER:
                    patch_msg.color.r = 0;
                    patch_msg.color.g = 0;
                    patch_msg.color.b = 1;
                    patch_msg.color.a = 1;
                    break;   
                case maps::grid::TraversabilityNodeBase::UNSET:
                    patch_msg.color.r = 1;
                    patch_msg.color.g = 1;
                    patch_msg.color.b = 0;
                    patch_msg.color.a = 1;
                    break;   
                case maps::grid::TraversabilityNodeBase::UNKNOWN:
                    patch_msg.color.r = 0.5;
                    patch_msg.color.g = 0;
                    patch_msg.color.b = 0.5;
                    patch_msg.color.a = 1;
                    break;   
            }

            msg.patches.push_back(patch_msg);
        }
    }
    grid_map_publisher->publish(grid_map);
    trav_map_publisher->publish(msg);

    RCLCPP_INFO_STREAM(this->get_logger(), "Published Grid Map");
   }
}