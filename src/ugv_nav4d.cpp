#include "ugv_nav4d.hpp"

using namespace rclcpp;

PathPlannerNode::PathPlannerNode()
    : Node("ugv_nav4d_node")
{
        // controller feedback (via TF)
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    timer_pose_samples = this->create_wall_timer(std::chrono::duration<double>(0.01), std::bind(&PathPlannerNode::read_pose_samples, this));
    sub_goal_pose = create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 1, bind(&PathPlannerNode::process_goal_request, this, std::placeholders::_1));
}

bool PathPlannerNode::read_pose_samples(){

    printConfigs();

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
    return true;    
}

void PathPlannerNode::process_goal_request(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
{
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