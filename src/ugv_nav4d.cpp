#include "ugv_nav4d.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <fstream>

using namespace rclcpp;
using namespace ugv_nav4d_ros2;

PathPlannerNode::PathPlannerNode()
    : Node("ugv_nav4d_node")
{
    declareParameters();

    // controller feedback (via TF)
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    timer_pose_samples = create_wall_timer(std::chrono::duration<double>(0.01), std::bind(&PathPlannerNode::read_pose_samples, this));
    sub_goal_pose = create_subscription<geometry_msgs::msg::PoseStamped>("/ugv_nav4d/goal_pose", 1, bind(&PathPlannerNode::process_goal_request, this, std::placeholders::_1));

    // Create a parameter subscriber that can be used to monitor parameter changes
    // (for this node's parameters as well as other nodes' parameters)
    param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // Set a callback for this node's parameter updates
    auto cb = [this](const rclcpp::Parameter & p) {
        RCLCPP_INFO(
          this->get_logger(), "cb: Received an update to parameter \"%s\" of type %s: \"%ld\"",
          p.get_name().c_str(),
          p.get_type_name().c_str(),
          p.as_int());
      };

    cb_handle = param_subscriber->add_parameter_callback("param_subscriber", cb);
    path_publisher = this->create_publisher<nav_msgs::msg::Path>("/ugv_nav4d/path", 10);
    trav_map_publisher = this->create_publisher<nav_msgs::msg::GridCells>("/ugv_nav4d/traversability_map", 10);

    configurePlanner();
    if (get_parameter("map_ply_path").as_string() != "default_value"){
        //Load map from file only when user explicity sets the parameter
        loadMls(get_parameter("map_ply_path").as_string());
    }
    else{
        //TODO: Need to get a map over a topic
    }
}

bool PathPlannerNode::read_pose_samples(){

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
        RCLCPP_INFO_STREAM(this->get_logger(), "Loading PLY: " << get_parameter("map_ply_path").as_string());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PLYReader plyReader;
        if(plyReader.read(path, *cloud) >= 0)
        {
            pcl::PointXYZ mi, ma; 
            pcl::getMinMax3D (*cloud, mi, ma); 

            //transform point cloud to zero (instead we could also use MlsMap::translate later but that seems to be broken?)
            Eigen::Affine3f pclTf = Eigen::Affine3f::Identity();
            pclTf.translation() << -mi.x, -mi.y, -mi.z;
            pcl::transformPointCloud (*cloud, *cloud, pclTf);
            
            pcl::getMinMax3D (*cloud, mi, ma); 
            RCLCPP_INFO_STREAM(this->get_logger(), "MIN: " << mi << ", MAX: " << ma);
        
            const double mls_res = get_parameter("grid_resolution").as_double();
            const double size_x = ma.x;
            const double size_y = ma.y;
            
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
        }
        return true;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Unabled to load mls. Unknown format");
    return false;
}

void PathPlannerNode::plan(){

    std::vector<trajectory_follower::SubTrajectory> trajectory2D, trajectory3D;
    base::Time time;
    time.microseconds = get_parameter("planningTime").as_int();

    if(!initialPatchAdded)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Setting Initial Patch of size: " << get_parameter("initialPatchRadius").as_double());
        planner->setInitialPatch(start_pose_rbs.getTransform(), get_parameter("initialPatchRadius").as_double());
        initialPatchAdded = true;
    }

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
            base::Pose2D start_pose = trajectory.getStartPose();
            base::Pose2D goal_pose  = trajectory.getGoalPose();

            // Fill current path point to a temporary variable.
            //TODO: Set the z path to the patch height to get a 3D Trajectory
            geometry_msgs::msg::PoseStamped tempPoint;
            tempPoint.pose.position.x= goal_pose.position.x();
            tempPoint.pose.position.y= goal_pose.position.y();
            tempPoint.pose.position.z= get_parameter("distToGround").as_double();

            // Add points to path.
            path_message.header.frame_id = get_parameter("world_frame").as_string();
            path_message.poses.push_back(tempPoint);
        }
        path_publisher->publish(path_message);
        RCLCPP_INFO_STREAM(this->get_logger(), "Published Path...");
    }
    publishTravMap();
}

void PathPlannerNode::declareParameters(){

    declare_parameter("map_ply_path", "default_value");

    declare_parameter("robot_frame", "robot");
    declare_parameter("world_frame", "map");
    declare_parameter("grid_resolution", 0.3);

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

void PathPlannerNode::publishTravMap(){

    RCLCPP_INFO_STREAM(this->get_logger(), "Getting Traversability Map");
    const auto& trav_map_3d = planner->getTraversabilityMap(); 

    nav_msgs::msg::GridCells grid_map;
    grid_map.header.frame_id = get_parameter("world_frame").as_string();
    grid_map.cell_height = trav_map_3d.getResolution().x();
    grid_map.cell_width  = trav_map_3d.getResolution().y();

    RCLCPP_INFO_STREAM(this->get_logger(), "Reading Traversability Map");
    for(const maps::grid::LevelList<traversability_generator3d::TravGenNode *> &l : trav_map_3d)
    {
        for(const traversability_generator3d::TravGenNode *n : l)
        {
            if (n->getType() == maps::grid::TraversabilityNodeBase::TRAVERSABLE){
                const Eigen::Vector3d& position = n->getVec3(trav_map_3d.getResolution().x());
                geometry_msgs::msg::Point cell_center;
                cell_center.x = position.x();
                cell_center.y = position.y();
                cell_center.z = position.z();
                grid_map.cells.push_back(cell_center);
            }
        }
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing Traversability Map...");
    trav_map_publisher->publish(grid_map);
    RCLCPP_INFO_STREAM(this->get_logger(), "Published Traversability Map");
}