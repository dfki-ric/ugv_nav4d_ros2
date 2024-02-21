#include "ugv_nav4d.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <fstream>

using namespace rclcpp;

PathPlannerNode::PathPlannerNode()
    : Node("ugv_nav4d_node")
{
    // controller feedback (via TF)
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    timer_pose_samples = this->create_wall_timer(std::chrono::duration<double>(0.01), std::bind(&PathPlannerNode::read_pose_samples, this));
    sub_goal_pose = create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 1, bind(&PathPlannerNode::process_goal_request, this, std::placeholders::_1));

    declare_parameter("robot_frame", "robot");
    declare_parameter("world_frame", "map");

    declare_parameter("dumpOnError", 0);
    declare_parameter("dumpOnSuccess", 0);
    declare_parameter("initialPatchRadius", 3.0);
    declare_parameter("microseconds", 50000000);

    declare_parameter("maxMotionCurveLength", 100);
    declare_parameter("minTurningRadius", 1);
    declare_parameter("multiplierBackward", 3);
    declare_parameter("multiplierBackwardTurn", 4);
    declare_parameter("multiplierForward", 1);
    declare_parameter("multiplierForwardTurn", 2);
    declare_parameter("multiplierLateral", 4);
    declare_parameter("multiplierLateralCurve", 4);
    declare_parameter("multiplierPointTurn", 3);
    declare_parameter("rotationSpeed", 1.0);
    declare_parameter("searchProgressSteps", 0.1);
    declare_parameter("searchRadius", 0.0);
    declare_parameter("translationSpeed", 1.0);

    declare_parameter("epsilonSteps", 2);
    declare_parameter("initialEpsilon", 64);
    declare_parameter("numThreads", 8);

    declare_parameter("cellSkipFactor", 3);
    declare_parameter("destinationCircleRadius", 10);
    declare_parameter("generateBackwardMotions", false);
    declare_parameter("generateForwardMotions", true);
    declare_parameter("generateLateralMotions", false);
    declare_parameter("generatePointTurnMotions", true);
    declare_parameter("gridSize", 0.3);
    declare_parameter("numAngles", 42);
    declare_parameter("numEndAngles", 21);
    declare_parameter("splineOrder", 4.0);

    declare_parameter("allowForwardDownhill", true);
    declare_parameter("costFunctionDist", 0.0);
    declare_parameter("distToGround", 0.0);
    declare_parameter("enableInclineLimitting", true);
    declare_parameter("gridResolution", 0.3);
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

    setupPlanner();
    planner.reset(new ugv_nav4d::Planner(splinePrimitiveConfig, traversabilityConfig, mobility, plannerConfig));
    loadMls("/opt/workspace/src/planning/ugv_nav4d/test_data/Plane1Mio.ply");
}

bool PathPlannerNode::read_pose_samples(){

    /*
    printConfigs();

    std::string robot_frame = get_parameter("robot_frame").as_string();
    std::string map_frame = get_parameter("map_frame").as_string();

    try{
        geometry_msgs::msg::TransformStamped t = tf_buffer->lookupTransform(map_frame, robot_frame, tf2::TimePointZero);
        pose_samples.pose.orientation = t.transform.rotation;
        pose_samples.pose.position.x =  t.transform.translation.x;
        pose_samples.pose.position.y =  t.transform.translation.y;
        pose_samples.pose.position.z =  t.transform.translation.z;
    }
    catch(const tf2::TransformException & ex){
        return false;
    }
    */

       //Dummy start for the time being
    pose_samples.pose.position.x = 2;
    pose_samples.pose.position.y = 2;
    pose_samples.pose.position.z = 0;

    pose_samples.pose.orientation.w = 1;
    pose_samples.pose.orientation.x = 0;
    pose_samples.pose.orientation.y = 0;
    pose_samples.pose.orientation.z = 0;

    return true;    
}

void PathPlannerNode::process_goal_request(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
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
    plan();
}

void PathPlannerNode::printConfigs(){
    std::cout << "SplinePrimitivesConfig: " << std::endl;
    std::cout << "gridSize: " << splinePrimitiveConfig.gridSize << std::endl;
    std::cout << "numAngles: " << splinePrimitiveConfig.numAngles << std::endl;
    std::cout << "numEndAngles: " << splinePrimitiveConfig.numEndAngles << std::endl;
    std::cout << "destinationCircleRadius: " << splinePrimitiveConfig.destinationCircleRadius << std::endl;
    std::cout << "cellSkipFactor: " << splinePrimitiveConfig.cellSkipFactor << std::endl;
    std::cout << "splineOrder: " << splinePrimitiveConfig.splineOrder << std::endl;
    std::cout << "generateForwardMotions: " << splinePrimitiveConfig.generateForwardMotions << std::endl;
    std::cout << "generateBackwardMotions: " << splinePrimitiveConfig.generateBackwardMotions << std::endl;
    std::cout << "generateLateralMotions: " << splinePrimitiveConfig.generateLateralMotions << std::endl;
    std::cout << "generatePointTurnMotions: " << splinePrimitiveConfig.generatePointTurnMotions << std::endl;
}

bool PathPlannerNode::loadMls(const std::string& path)
{
    std::ifstream fileIn(path);       

    if(path.find(".ply") != std::string::npos)
    {
        std::cout << "Loading PLY" << std::endl;
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
            std::cout << "MIN: " << mi << ", MAX: " << ma << std::endl;
        
            const double mls_res = 0.3;
            const double size_x = ma.x;
            const double size_y = ma.y;
            
            const maps::grid::Vector2ui numCells(size_x / mls_res + 1, size_y / mls_res + 1);
            std::cout << "NUM CELLS: " << numCells << std::endl;
            
            maps::grid::MLSConfig cfg;
            cfg.gapSize = 0.1;
            mlsMap = maps::grid::MLSMapSloped(numCells, maps::grid::Vector2d(mls_res, mls_res), cfg);
            mlsMap.mergePointCloud(*cloud, base::Transform3d::Identity());
            planner->updateMap(mlsMap);
        }
        return true;
    }
    std::cerr << "Unabled to load mls. Unknown format" << std::endl;
    return false;
}

void PathPlannerNode::plan(){
    std::vector<trajectory_follower::SubTrajectory> trajectory2D, trajectory3D;

    base::Time time;
    time.microseconds = 5000000;

    std::cout << "Planning..." << std::endl;
    ugv_nav4d::Planner::PLANNING_RESULT res = planner->plan(time, start_pose_rbs, goal_pose_rbs, trajectory2D, trajectory3D, false, false);
    
    switch(res)
    {
        case ugv_nav4d::Planner::FOUND_SOLUTION:
            std::cout << "FOUND_SOLUTION" << std::endl;
            break;
        case ugv_nav4d::Planner::GOAL_INVALID:
            std::cout << "GOAL_INVALID" << std::endl;
            break;
        case ugv_nav4d::Planner::START_INVALID:
            std::cout << "START_INVALID" << std::endl;
            break;
        case ugv_nav4d::Planner::INTERNAL_ERROR:
            std::cout << "INTERNAL_ERROR" << std::endl;
            break;
        case ugv_nav4d::Planner::NO_SOLUTION:
            std::cout << "NO_SOLUTION" << std::endl;
            break;
        case ugv_nav4d::Planner::NO_MAP:
            std::cout << "NO_MAP" << std::endl;
            break;
    }
}

void PathPlannerNode::setupPlanner()
{
    double res = 0.3;

    splinePrimitiveConfig.gridSize = res;
    splinePrimitiveConfig.numAngles = 24;
    splinePrimitiveConfig.numEndAngles = 12;
    splinePrimitiveConfig.destinationCircleRadius = 5;
    splinePrimitiveConfig.cellSkipFactor = 1.0;
    splinePrimitiveConfig.generatePointTurnMotions = true;
    splinePrimitiveConfig.generateLateralMotions = false;
    splinePrimitiveConfig.generateBackwardMotions = true;
    splinePrimitiveConfig.splineOrder = 4;
    
    mobility.translationSpeed = 0.2;
    mobility.rotationSpeed = 0.6;
    mobility.minTurningRadius = 0.2; // increase this to reduce the number of available motion primitives
    mobility.searchRadius = 0.0;
    mobility.multiplierForward = 1;
    mobility.multiplierBackward = 1;
    mobility.multiplierLateral = 3;
    mobility.multiplierBackwardTurn = 1;
    mobility.multiplierForwardTurn = 1;
    mobility.multiplierPointTurn = 3;
     
    traversabilityConfig.gridResolution = res;
    traversabilityConfig.maxSlope = 0.57; //40.0/180.0 * M_PI;
    traversabilityConfig.maxStepHeight = 0.3; //space below robot
    traversabilityConfig.robotSizeX = 0.9;
    traversabilityConfig.robotSizeY =  0.5;
    traversabilityConfig.robotHeight = 0.9; //incl space below body
    traversabilityConfig.slopeMetricScale = 0.0;
    traversabilityConfig.slopeMetric = traversability_generator3d::SlopeMetric::NONE;
    traversabilityConfig.inclineLimittingMinSlope = 0.22; // 10.0 * M_PI/180.0;
    traversabilityConfig.inclineLimittingLimit = 0.43;// 5.0 * M_PI/180.0;
    traversabilityConfig.costFunctionDist = 0.0;
    traversabilityConfig.distToGround = 0.0;
    traversabilityConfig.minTraversablePercentage = 0.5;
    traversabilityConfig.allowForwardDownhill = true;

    plannerConfig.epsilonSteps = 2.0;
    plannerConfig.initialEpsilon = 20.0;
    plannerConfig.numThreads = 4;    
}