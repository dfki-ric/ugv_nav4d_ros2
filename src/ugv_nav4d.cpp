#include <rclcpp/rclcpp.hpp>

// #include <geometry_msgs/msg/twist.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <std_msgs/msg/float64_multi_array.hpp>


// class MotionController: public rclcpp::Node
// {
//   public:
//     MotionController()
//     : Node("ugv_nav4d")
//     {
//         // declare the configuration parameter for node
//         this->declare_parameter("frontWheelRadius", 0.261);
//         this->declare_parameter("frontTrackWidth", 0.54344);
//         this->declare_parameter("rearWheelRadius", 0.261);
//         this->declare_parameter("rearTrackWidth", 0.5427);
//         this->declare_parameter("roverLength", 0.5777);
//         this->declare_parameter("pointTurnTransSpeedTol", 0.000001);
//         // this parameter is irrelevant for use
//         // it was needed to set the names in base::samples::Joints command
//         this->declare_parameter("wheel_joints_front_left", "motor_wheel_front_left");
//         this->declare_parameter("wheel_joints_front_right", "motor_wheel_front_right");
//         this->declare_parameter("wheel_joints_rear_left", "motor_wheel_rear_left");
//         this->declare_parameter("wheel_joints_rear_right", "motor_wheel_rear_right");

//         // set up motion controller configuration from node parameter
//         coyote::MotionController::Config mc_config;
//         mc_config.frontWheelRadius = this->get_parameter("frontWheelRadius").as_double();
//         mc_config.frontTrackWidth = this->get_parameter("frontTrackWidth").as_double();
//         mc_config.rearWheelRadius = this->get_parameter("rearWheelRadius").as_double();
//         mc_config.rearTrackWidth = this->get_parameter("rearTrackWidth").as_double();
//         mc_config.roverLength = this->get_parameter("roverLength").as_double();
//         mc_config.pointTurnTransSpeedTol = this->get_parameter("pointTurnTransSpeedTol").as_double();
//         mc_config.wheels.front_left = this->get_parameter("wheel_joints_front_left").as_string();
//         mc_config.wheels.front_right = this->get_parameter("wheel_joints_front_right").as_string();
//         mc_config.wheels.rear_left = this->get_parameter("wheel_joints_rear_left").as_string();
//         mc_config.wheels.rear_right = this->get_parameter("wheel_joints_rear_right").as_string();

        
//         current_joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>
//                                 ("joint_states", 10, std::bind(&MotionController::read_current_joint_state, this, std::placeholders::_1));

//         motion2d_cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>
//                                 ("cmd_vel", 10, std::bind(&MotionController::read_motion2d_cmd, this, std::placeholders::_1));

//         joint_cmd_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_cmd", 10);
//     }

//   private:
//     void read_current_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg) const
//     {
//     }

//     void read_motion2d_cmd(const geometry_msgs::msg::Twist::SharedPtr msg) const
//     {
//         base::commands::Motion2D givenSpeedCommand;

//         if (msg->linear.z != 0) {
//             RCLCPP_ERROR(this->get_logger(), "linear Z is not zero");
//             return;
//         }

//         if (msg->angular.x != 0 || msg->angular.y != 0) {
//             RCLCPP_ERROR(this->get_logger(), "angular X or Y is not zero");
//             return;
//         }

//         givenSpeedCommand.translation = std::sqrt(msg->linear.x * msg->linear.x + msg->linear.y * msg->linear.y);
//         givenSpeedCommand.rotation = msg->angular.z;
//         givenSpeedCommand.heading.rad = std::atan2(msg->linear.y, msg->linear.x);

//         motion_controller->calcWheelSpeeds(givenSpeedCommand);

//         base::samples::Joints cmd = motion_controller->getCurrentSpeeds();

//         std_msgs::msg::Float64MultiArray commands;
//         commands.data.push_back(cmd[0].speed);
//         commands.data.push_back(cmd[1].speed);
//         commands.data.push_back(cmd[2].speed);
//         commands.data.push_back(cmd[3].speed);

//         joint_cmd_pub->publish(commands);
//     }

//     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion2d_cmd_sub;
//     rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr current_joint_state_sub;

//     rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub;

// };

int main(int argc, char * argv[])
{
    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<MotionController>());
    // rclcpp::shutdown();
	return 0;
}