#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include <ugv_nav4d/PlannerGui.h>
#include "ugv_nav4d_ros2.hpp"
#include <thread>

#pragma message("Compiler version: " __VERSION__)
#pragma message("C++ standard (__cplusplus): "  )
#if defined(__cplusplus)
  #if __cplusplus == 201703L
    #pragma message("Standard value is C++17")
  #elif __cplusplus == 202002L
    #pragma message("Standard value is C++20")
  #elif __cplusplus == 201402L
    #pragma message("Standard value is C++14")
  #elif __cplusplus == 201103L
    #pragma message("Standard value is C++11")
  #else
    #pragma message("Standard value is unknown")
  #endif
#endif

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Initialize Qt Application
    QApplication app(argc, argv);
    
    // Create the ROS 2 planner node
    auto ros_node = std::make_shared<ugv_nav4d_ros2::PathPlannerNode>();

    // Populate config structs from params.yaml (loaded on parameter server)
    // This must be called before getting the configs
    ros_node->loadConfigsFromParameters();

    // Create the Planner GUI without loading config from file (autoLoadMls=false, loadConfigFromFile=false)
    PlannerGui gui(argc, argv, false, false);

    // Load configuration from ROS 2 node into GUI
    gui.splineConfig = ros_node->getSplineConfig();
    gui.mobilityConfig = ros_node->getMobilityConfig();
    gui.travConfig = ros_node->getTraversabilityConfig();
    gui.plannerConfig = ros_node->getPlannerConfig();

    // Rebuild planner using the node's configs (loadConfigFromFile=false)
    gui.setupPlanner(argc, argv, false, false);

    // Setup callback to reflect the node's map/planner state in the GUI status label
    ros_node->registerStatusCallback([&gui](const std::string& msg) {
        QMetaObject::invokeMethod(&gui, [&gui, msg]() {
            gui.setStatusMessage(msg);
        }, Qt::QueuedConnection);
    });

    // Continuously track the robot start pose (TF / topic) in the GUI start marker
    ros_node->registerPoseUpdateCallback([&gui](const base::Pose& startPose) {
        QMetaObject::invokeMethod(&gui, [&gui, startPose]() {
            gui.updateStartPose(startPose);
        }, Qt::QueuedConnection);
    });

    // Setup callback to update GUI maps when ROS 2 Node builds/loads maps
    ros_node->registerMapUpdateCallback([&gui, ros_node]() {
        auto mlsMap = ros_node->getMLSMap();
        auto travMap = ros_node->getTraversabilityMap();
        
        if (mlsMap) {
            QMetaObject::invokeMethod(&gui, [&gui, mlsMap]() {
                gui.updateMlsMap(*mlsMap);
            }, Qt::QueuedConnection);
        }
        if (travMap) {
            QMetaObject::invokeMethod(&gui, [&gui, travMap]() {
                gui.updateTravMap(*travMap);
            }, Qt::QueuedConnection);
        }
        // Show the robot's actual start pose (from TF / start_pose topic) as the start marker.
        base::Pose startPose = ros_node->getStartPose();
        QMetaObject::invokeMethod(&gui, [&gui, startPose]() {
            gui.updateStartPose(startPose);
        }, Qt::QueuedConnection);
    });
    
    // Setup callback so GUI "Update Params" pushes config changes into the ROS 2 node
    gui.configUpdateCallback = [&gui, ros_node]() {
        ros_node->updateParametersFromConfigs(gui.splineConfig, gui.mobilityConfig,
                                              gui.travConfig, gui.plannerConfig);
    };

    // Setup callback to trigger ROS 2 planning when "Plan" is requested from the GUI
    gui.customPlanCallback = [&gui, ros_node](const base::Pose& start, const base::Pose& goal) {
        printf("DEBUG: customPlanCallback started\n");
        ros_node->triggerPlanningFromGUI(start, goal);
        printf("DEBUG: triggerPlanningFromGUI returned\n");

        // Reflect the actual start pose used by the node (from TF / topic) in the GUI marker.
        base::Pose startPose = ros_node->getStartPose();
        QMetaObject::invokeMethod(&gui, [&gui, startPose]() {
            gui.updateStartPose(startPose);
        }, Qt::QueuedConnection);

        auto path2D = ros_node->getLatestTrajectory2D();
        auto path3D = ros_node->getLatestTrajectory3D();
        auto res = ros_node->getLatestPlanningResult();
        printf("DEBUG: got latest trajectories and result: %d\n", (int)res);
        
        QMetaObject::invokeMethod(&gui, [&gui, path2D, path3D, res]() {
            printf("DEBUG: inside QMetaObject::invokeMethod lambda\n");
            gui.showPath(path2D, path3D, res);
            printf("DEBUG: gui.showPath returned inside lambda\n");
        }, Qt::QueuedConnection);
        printf("DEBUG: customPlanCallback returning\n");
    };
    
    // Show GUI Window
    gui.show();
    
    // Spin ROS 2 in a background executor thread
    std::thread ros_thread([ros_node]() {
        rclcpp::spin(ros_node);
    });
    ros_thread.detach();
    
    // Run the main Qt event loop
    int ret = app.exec();
    
    // Shutdown ROS 2 on exit
    rclcpp::shutdown();
    
    return ret;
}
