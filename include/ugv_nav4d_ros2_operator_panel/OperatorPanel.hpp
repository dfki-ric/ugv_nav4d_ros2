#ifndef UGV_NAV4D_ROS2_OPERATOR_PANEL_HPP_
#define UGV_NAV4D_ROS2_OPERATOR_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "ugv_nav4d_ros2/srv/edit_waypoint.hpp"
#include "ugv_nav4d_ros2/srv/delete_forbidden_zone.hpp"
#include "ugv_nav4d_ros2/msg/mission_status.hpp"
#include "ugv_nav4d_ros2/msg/route_risk.hpp"
#include "ugv_nav4d_ros2/msg/system_health.hpp"

#include <rviz_common/panel.hpp>

class QLabel;
class QComboBox;
class QPushButton;
class QSpinBox;

namespace ugv_nav4d_ros2
{
namespace ugv_nav4d_ros2_operator_panel
{

/**
 * Field operator panel for the ugv_nav4d planner: shows the live planner
 * status and offers one-click access to the waypoint-queue and map services.
 */
class OperatorPanel : public rviz_common::Panel
{
    Q_OBJECT

public:
    explicit OperatorPanel(QWidget* parent = nullptr);
    ~OperatorPanel() override;

    void onInitialize() override;

private Q_SLOTS:
    void onStopExecution();
    void onPauseExecution();
    void onResumeExecution();
    void onRecoverMission();
    void onReplanMission();
    void onExecutePath();
    void onDiscardPath();
    void onDeleteWaypoint();
    void onDeleteZone();
    void onPlanReturn();
    void onReturnModeChanged(int index);
    void onClearWaypoints();
    void onUndoWaypoint();
    void onClearZones();
    void onUndoZone();
    void onSaveMap();
    void onRepublishMaps();
    void onRegenerateMaps();
    void onSaveMission();
    void onLoadMission();

private:
    void callTrigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client,
                     const QString& actionName);
    void setStatusText(const QString& text);
    void updateExecuteEnabled();
    void waitForRecoveryStop(int attempts_remaining);
    void requestNativeRecovery();
    void finishRecovery(const QString& status);

    QLabel* status_label_{nullptr};
    QLabel* execution_label_{nullptr};
    QLabel* risk_label_{nullptr};
    QLabel* health_label_{nullptr};
    QLabel* readiness_label_{nullptr};
    QLabel* inspection_label_{nullptr};
    QPushButton* stop_button_{nullptr};
    QPushButton* pause_button_{nullptr};
    QPushButton* resume_button_{nullptr};
    QPushButton* recover_button_{nullptr};
    QPushButton* replan_button_{nullptr};
    QPushButton* execute_path_button_{nullptr};
    QPushButton* discard_path_button_{nullptr};
    QSpinBox* waypoint_index_spin_{nullptr};
    QPushButton* delete_waypoint_button_{nullptr};
    QSpinBox* zone_index_spin_{nullptr};
    QPushButton* delete_zone_button_{nullptr};
    QPushButton* plan_return_button_{nullptr};
    QComboBox* return_mode_combo_{nullptr};
    QPushButton* clear_waypoints_button_{nullptr};
    QPushButton* undo_waypoint_button_{nullptr};
    QPushButton* clear_zones_button_{nullptr};
    QPushButton* undo_zone_button_{nullptr};
    QPushButton* save_map_button_{nullptr};
    QPushButton* republish_maps_button_{nullptr};
    QPushButton* regenerate_maps_button_{nullptr};
    QPushButton* save_mission_button_{nullptr};
    QPushButton* load_mission_button_{nullptr};

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    rclcpp::Subscription<ugv_nav4d_ros2::msg::MissionStatus>::SharedPtr execution_status_sub_;
    rclcpp::Subscription<ugv_nav4d_ros2::msg::RouteRisk>::SharedPtr route_risk_sub_;
    rclcpp::Subscription<ugv_nav4d_ros2::msg::SystemHealth>::SharedPtr system_health_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr inspection_result_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_execution_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_execution_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resume_execution_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr replan_mission_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr native_recovery_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr execute_path_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr discard_path_client_;
    rclcpp::Client<ugv_nav4d_ros2::srv::EditWaypoint>::SharedPtr edit_waypoint_client_;
    rclcpp::Client<ugv_nav4d_ros2::srv::DeleteForbiddenZone>::SharedPtr delete_zone_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr plan_return_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_return_forward_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_waypoints_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr remove_last_waypoint_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_zones_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr undo_zone_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_map_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr map_publish_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr regenerate_maps_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_mission_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr load_mission_client_;
    bool health_ok_{false};
    bool route_ready_{false};
    uint8_t execution_state_{ugv_nav4d_ros2::msg::MissionStatus::IDLE};
    bool execution_can_resume_{false};
    bool recovery_in_progress_{false};
};

} // namespace ugv_nav4d_ros2_operator_panel
} // namespace ugv_nav4d_ros2

#endif // UGV_NAV4D_ROS2_OPERATOR_PANEL_HPP_
