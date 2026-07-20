#include "OperatorPanel.hpp"

#include <algorithm>

#include <QGridLayout>
#include <QComboBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QStringList>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace ugv_nav4d_ros2
{
namespace ugv_nav4d_ros2_operator_panel
{

OperatorPanel::OperatorPanel(QWidget* parent)
: rviz_common::Panel(parent)
{
    auto* layout = new QVBoxLayout;

    // Safety-critical control stays above all variable-height status text so
    // long planner/health messages can never push it out of the panel viewport.
    stop_button_ = new QPushButton("STOP");
    stop_button_->setMinimumHeight(44);
    stop_button_->setStyleSheet("QPushButton { font-weight: bold; font-size: 16px; background-color: #b71c1c; color: white; padding: 6px; }");
    layout->addWidget(stop_button_);

    status_label_ = new QLabel("(waiting for planner status)");
    status_label_->setWordWrap(true);
    status_label_->setMaximumHeight(60);
    status_label_->setStyleSheet("QLabel { font-weight: bold; }");
    layout->addWidget(new QLabel("Planner status:"));
    layout->addWidget(status_label_);

    execution_label_ = new QLabel("Execution: idle");
    execution_label_->setWordWrap(true);
    execution_label_->setMaximumHeight(60);
    risk_label_ = new QLabel("Route risk: no path");
    risk_label_->setWordWrap(true);
    risk_label_->setMaximumHeight(60);
    health_label_ = new QLabel("System health: waiting");
    health_label_->setWordWrap(true);
    health_label_->setMaximumHeight(60);
    readiness_label_ = new QLabel("Readiness: waiting");
    readiness_label_->setWordWrap(true);
    readiness_label_->setMaximumHeight(180);
    readiness_label_->setStyleSheet("QLabel { font-family: monospace; }");
    inspection_label_ = new QLabel("Inspection: select Inspect Traversability and click a map patch");
    inspection_label_->setWordWrap(true);
    inspection_label_->setMaximumHeight(60);
    inspection_label_->setStyleSheet("QLabel { color: #69a8ff; }");
    layout->addWidget(execution_label_);
    layout->addWidget(risk_label_);
    layout->addWidget(health_label_);
    layout->addWidget(readiness_label_);
    layout->addWidget(inspection_label_);

    auto* buttons = new QGridLayout;
    pause_button_ = new QPushButton("Pause");
    resume_button_ = new QPushButton("Resume");
    recover_button_ = new QPushButton("Recover from obstacle");
    recover_button_->setStyleSheet(
        "QPushButton { font-weight: bold; background-color: #ef6c00; color: white; padding: 6px; }");
    recover_button_->setToolTip(
        "Safely pause active execution, then use ugv_nav4d's native out-of-obstacle recovery to create an approval-gated rescue trajectory.");
    replan_button_ = new QPushButton("Replan from robot");
    execute_path_button_ = new QPushButton("Execute path");
    execute_path_button_->setStyleSheet("QPushButton { font-weight: bold; background-color: #2e7d32; color: white; }");
    discard_path_button_ = new QPushButton("Discard path");
    clear_waypoints_button_ = new QPushButton("Clear waypoints");
    undo_waypoint_button_ = new QPushButton("Undo last waypoint");
    clear_zones_button_ = new QPushButton("Clear zones");
    undo_zone_button_ = new QPushButton("Undo last zone");
    save_map_button_ = new QPushButton("Save MLS map");
    republish_maps_button_ = new QPushButton("Republish maps");
    regenerate_maps_button_ = new QPushButton("Regenerate maps");
    buttons->addWidget(pause_button_, 0, 0);
    buttons->addWidget(resume_button_, 0, 1);
    buttons->addWidget(recover_button_, 1, 0, 1, 2);
    buttons->addWidget(replan_button_, 2, 0, 1, 2);
    buttons->addWidget(execute_path_button_, 3, 0);
    buttons->addWidget(discard_path_button_, 3, 1);
    buttons->addWidget(clear_waypoints_button_, 4, 0);
    buttons->addWidget(undo_waypoint_button_, 4, 1);
    buttons->addWidget(clear_zones_button_, 5, 0);
    buttons->addWidget(undo_zone_button_, 5, 1);
    buttons->addWidget(save_map_button_, 6, 0);
    buttons->addWidget(republish_maps_button_, 6, 1);
    buttons->addWidget(regenerate_maps_button_, 7, 0, 1, 2);
    save_mission_button_ = new QPushButton("Save mission");
    load_mission_button_ = new QPushButton("Load mission");
    buttons->addWidget(save_mission_button_, 8, 0);
    buttons->addWidget(load_mission_button_, 8, 1);
    plan_return_button_ = new QPushButton("Plan return");
    layout->addLayout(buttons);

    auto* return_group = new QGroupBox("Plan return options");
    auto* return_layout = new QVBoxLayout;
    return_mode_combo_ = new QComboBox;
    return_mode_combo_->addItem("Turn around and drive forward");
    return_mode_combo_->addItem("Preserve headings / allow reverse");
    return_mode_combo_->setToolTip(
        "Select how waypoint headings are handled when planning the return mission.");
    return_layout->addWidget(return_mode_combo_);
    return_layout->addWidget(plan_return_button_);
    return_group->setLayout(return_layout);
    layout->addWidget(return_group);

    auto* wp_row = new QHBoxLayout;
    wp_row->addWidget(new QLabel("WP #"));
    waypoint_index_spin_ = new QSpinBox;
    waypoint_index_spin_->setMinimum(1);
    waypoint_index_spin_->setMaximum(999);
    wp_row->addWidget(waypoint_index_spin_);
    delete_waypoint_button_ = new QPushButton("Delete waypoint");
    wp_row->addWidget(delete_waypoint_button_);
    wp_row->addWidget(new QLabel("Zone #"));
    zone_index_spin_ = new QSpinBox;
    zone_index_spin_->setMinimum(1);
    zone_index_spin_->setMaximum(999);
    wp_row->addWidget(zone_index_spin_);
    delete_zone_button_ = new QPushButton("Delete zone");
    wp_row->addWidget(delete_zone_button_);
    layout->addLayout(wp_row);

    layout->addStretch();
    setLayout(layout);

    connect(stop_button_, &QPushButton::clicked, this, &OperatorPanel::onStopExecution);
    connect(pause_button_, &QPushButton::clicked, this, &OperatorPanel::onPauseExecution);
    connect(resume_button_, &QPushButton::clicked, this, &OperatorPanel::onResumeExecution);
    connect(recover_button_, &QPushButton::clicked, this, &OperatorPanel::onRecoverMission);
    connect(replan_button_, &QPushButton::clicked, this, &OperatorPanel::onReplanMission);
    connect(execute_path_button_, &QPushButton::clicked, this, &OperatorPanel::onExecutePath);
    connect(discard_path_button_, &QPushButton::clicked, this, &OperatorPanel::onDiscardPath);
    connect(delete_waypoint_button_, &QPushButton::clicked, this, &OperatorPanel::onDeleteWaypoint);
    connect(delete_zone_button_, &QPushButton::clicked, this, &OperatorPanel::onDeleteZone);
    connect(plan_return_button_, &QPushButton::clicked, this, &OperatorPanel::onPlanReturn);
    connect(return_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &OperatorPanel::onReturnModeChanged);
    connect(clear_waypoints_button_, &QPushButton::clicked, this, &OperatorPanel::onClearWaypoints);
    connect(undo_waypoint_button_, &QPushButton::clicked, this, &OperatorPanel::onUndoWaypoint);
    connect(clear_zones_button_, &QPushButton::clicked, this, &OperatorPanel::onClearZones);
    connect(undo_zone_button_, &QPushButton::clicked, this, &OperatorPanel::onUndoZone);
    connect(save_map_button_, &QPushButton::clicked, this, &OperatorPanel::onSaveMap);
    connect(republish_maps_button_, &QPushButton::clicked, this, &OperatorPanel::onRepublishMaps);
    connect(regenerate_maps_button_, &QPushButton::clicked, this, &OperatorPanel::onRegenerateMaps);
    connect(save_mission_button_, &QPushButton::clicked, this, &OperatorPanel::onSaveMission);
    connect(load_mission_button_, &QPushButton::clicked, this, &OperatorPanel::onLoadMission);
    updateExecuteEnabled();
}

OperatorPanel::~OperatorPanel() = default;

void OperatorPanel::onInitialize()
{
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    // Matches the transient_local publisher in the planner node, so the panel
    // shows the current state immediately after being opened.
    status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/ugv_nav4d_ros2/status", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            setStatusText(QString::fromStdString(msg->data));
        });
    execution_status_sub_ = node_->create_subscription<ugv_nav4d_ros2::msg::MissionStatus>(
        "/ugv_nav4d_ros2/execution_status", 10,
        [this](const ugv_nav4d_ros2::msg::MissionStatus::SharedPtr msg)
        {
            const QString segment_status =
                msg->current_segment == 0 && msg->total_segments > 0
                    ? QString("Not started — %1 segment%2")
                        .arg(msg->total_segments)
                        .arg(msg->total_segments == 1 ? "" : "s")
                    : QString("%1/%2").arg(msg->current_segment).arg(msg->total_segments);
            const QString text = QString("Execution: %1 — %2 [%3], %4 m remaining")
                .arg(QString::fromStdString(msg->state_name))
                .arg(QString::fromStdString(msg->summary))
                .arg(segment_status)
                .arg(msg->distance_remaining, 0, 'f', 1);
            QMetaObject::invokeMethod(this, [this, text, state = msg->state,
                                                   can_resume = msg->can_resume]() {
                execution_label_->setText(text);
                execution_label_->setToolTip(text);
                execution_state_ = state;
                execution_can_resume_ = can_resume;
            }, Qt::QueuedConnection);
        });
    route_risk_sub_ = node_->create_subscription<ugv_nav4d_ros2::msg::RouteRisk>(
        "/ugv_nav4d_ros2/route_risk", rclcpp::QoS(1).transient_local(),
        [this](const ugv_nav4d_ros2::msg::RouteRisk::SharedPtr msg)
        {
            const QString text = "Route risk: " + QString::fromStdString(msg->summary);
            QMetaObject::invokeMethod(this, [this, text, valid = msg->valid]() {
                risk_label_->setText(text);
                risk_label_->setToolTip(text);
                route_ready_ = valid;
                updateExecuteEnabled();
            }, Qt::QueuedConnection);
        });
    system_health_sub_ = node_->create_subscription<ugv_nav4d_ros2::msg::SystemHealth>(
        "/ugv_nav4d_ros2/system_health", 10,
        [this](const ugv_nav4d_ros2::msg::SystemHealth::SharedPtr msg)
        {
            const QString text = "System health: " + QString::fromStdString(msg->summary);
            QStringList rows;
            QStringList tooltip_rows;
            const size_t count = std::min({
                msg->readiness_names.size(),
                msg->readiness_levels.size(),
                msg->readiness_messages.size()});
            for (size_t i = 0; i < count; ++i) {
                const uint8_t item_level = msg->readiness_levels[i];
                const QString badge =
                    item_level == ugv_nav4d_ros2::msg::SystemHealth::OK ? "OK  " :
                    item_level == ugv_nav4d_ros2::msg::SystemHealth::WARN ? "WARN" : "ERR ";
                const QString name = QString::fromStdString(msg->readiness_names[i]);
                const QString detail = QString::fromStdString(msg->readiness_messages[i]);
                rows << QString("%1  %2").arg(badge, name);
                tooltip_rows << QString("%1  %2: %3").arg(badge, name, detail);
            }
            const QString readiness_text = rows.isEmpty()
                ? QString("Readiness: waiting")
                : QString("Readiness: %1\n%2")
                    .arg(msg->ready_to_execute ? "ready" : "not ready")
                    .arg(rows.join('\n'));
            const QString readiness_tooltip = tooltip_rows.isEmpty()
                ? readiness_text
                : QString("Ready to execute: %1\n%2")
                    .arg(msg->ready_to_execute ? "yes" : "no")
                    .arg(tooltip_rows.join('\n'));
            QMetaObject::invokeMethod(this, [this, text, level = msg->level,
                                             readiness_text, readiness_tooltip,
                                             ready = msg->ready_to_execute]() {
                health_label_->setText(text);
                health_label_->setToolTip(text);
                health_label_->setStyleSheet(level == ugv_nav4d_ros2::msg::SystemHealth::OK
                    ? "QLabel { color: #2e7d32; }" : "QLabel { color: #c62828; font-weight: bold; }");
                readiness_label_->setText(readiness_text);
                readiness_label_->setToolTip(readiness_tooltip);
                readiness_label_->setStyleSheet(ready
                    ? "QLabel { color: #2e7d32; font-family: monospace; }"
                    : "QLabel { color: #c62828; font-family: monospace; }");
                health_ok_ = level != ugv_nav4d_ros2::msg::SystemHealth::ERROR;
                updateExecuteEnabled();
            }, Qt::QueuedConnection);
        });
    inspection_result_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/ugv_nav4d_ros2/inspection_result", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            const QString text = "Inspection: " + QString::fromStdString(msg->data);
            QMetaObject::invokeMethod(inspection_label_,
                [label = inspection_label_, text]() { label->setText(text); label->setToolTip(text); },
                Qt::QueuedConnection);
        });

    stop_execution_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/stop_execution");
    pause_execution_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/pause_execution");
    resume_execution_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/resume_execution");
    replan_mission_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/replan_current_mission");
    native_recovery_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/recover_out_of_obstacle");
    execute_path_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/execute_path");
    discard_path_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/discard_path");
    edit_waypoint_client_ = node_->create_client<ugv_nav4d_ros2::srv::EditWaypoint>("/ugv_nav4d_ros2/edit_waypoint");
    delete_zone_client_ = node_->create_client<ugv_nav4d_ros2::srv::DeleteForbiddenZone>("/ugv_nav4d_ros2/delete_forbidden_zone");
    plan_return_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/plan_return");
    set_return_forward_client_ = node_->create_client<std_srvs::srv::SetBool>("/ugv_nav4d_ros2/set_return_forward");
    clear_waypoints_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/clear_waypoints");
    remove_last_waypoint_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/remove_last_waypoint");
    clear_zones_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/clear_forbidden_zones");
    undo_zone_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/remove_last_forbidden_zone");
    save_map_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/save_mls_map");
    map_publish_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/map_publish");
    regenerate_maps_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/regenerate_maps");
    save_mission_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/save_mission");
    load_mission_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/load_mission");

}

void OperatorPanel::setStatusText(const QString& text)
{
    // Subscription and service callbacks run on the ROS spin thread; Qt widgets
    // must only be touched from the GUI thread.
    QMetaObject::invokeMethod(status_label_, [label = status_label_, text]()
    {
        label->setText(text);
        label->setToolTip(text);
    }, Qt::QueuedConnection);
}

void OperatorPanel::updateExecuteEnabled()
{
    execute_path_button_->setEnabled(health_ok_ && route_ready_);
    execute_path_button_->setToolTip(health_ok_ && route_ready_
        ? "Send the reviewed path to the controller."
        : "Execution requires a valid route and non-error system health.");
}

void OperatorPanel::callTrigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client,
                                const QString& actionName)
{
    if (!client || !node_)
    {
        return;
    }
    if (!client->service_is_ready())
    {
        setStatusText(actionName + ": service unavailable (is the planner node running?)");
        return;
    }

    setStatusText(actionName + " requested...");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(request,
        [this, actionName](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
        {
            const auto& response = future.get();
            QString text = QString::fromStdString(response->message);
            if (text.isEmpty())
            {
                text = actionName + (response->success ? " done" : " failed");
            }
            else if (!response->success)
            {
                text = actionName + " failed: " + text;
            }
            setStatusText(text);
        });
}

void OperatorPanel::onStopExecution()
{
    if (recovery_in_progress_)
    {
        recovery_in_progress_ = false;
        recover_button_->setEnabled(true);
    }
    callTrigger(stop_execution_client_, "STOP");
}

void OperatorPanel::onPauseExecution()
{
    callTrigger(pause_execution_client_, "Pause");
}

void OperatorPanel::onResumeExecution()
{
    callTrigger(resume_execution_client_, "Resume");
}

void OperatorPanel::onRecoverMission()
{
    if (recovery_in_progress_)
    {
        setStatusText("Recovery is already in progress.");
        return;
    }
    if (!node_ || !native_recovery_client_ || !native_recovery_client_->service_is_ready())
    {
        setStatusText("Recovery unavailable: native recovery service is not ready.");
        return;
    }

    recovery_in_progress_ = true;
    recover_button_->setEnabled(false);
    route_ready_ = false;
    updateExecuteEnabled();

    if (execution_state_ == ugv_nav4d_ros2::msg::MissionStatus::PAUSED &&
        !execution_can_resume_)
    {
        setStatusText("Recovery: waiting for the controller to confirm the pending pause...");
        waitForRecoveryStop(20);
        return;
    }
    if (execution_state_ != ugv_nav4d_ros2::msg::MissionStatus::EXECUTING)
    {
        requestNativeRecovery();
        return;
    }
    if (!pause_execution_client_ || !pause_execution_client_->service_is_ready())
    {
        finishRecovery("Recovery failed: pause service is unavailable; use STOP before replanning.");
        return;
    }

    setStatusText("Recovery: requesting a controlled pause...");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    pause_execution_client_->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
        {
            const auto response = future.get();
            QMetaObject::invokeMethod(this, [this, success = response->success,
                                                   message = response->message]()
            {
                if (!recovery_in_progress_)
                {
                    return;
                }
                if (!success)
                {
                    if (execution_state_ == ugv_nav4d_ros2::msg::MissionStatus::PAUSED &&
                        !execution_can_resume_)
                    {
                        waitForRecoveryStop(20);
                        return;
                    }
                    if (execution_state_ != ugv_nav4d_ros2::msg::MissionStatus::EXECUTING)
                    {
                        requestNativeRecovery();
                        return;
                    }
                    finishRecovery("Recovery pause failed: " + QString::fromStdString(message));
                    return;
                }
                setStatusText("Recovery: waiting for the controller to confirm pause...");
                waitForRecoveryStop(20);  // 20 * 250 ms = 5 s maximum wait.
            }, Qt::QueuedConnection);
        });
}

void OperatorPanel::waitForRecoveryStop(int attempts_remaining)
{
    if (!recovery_in_progress_)
    {
        return;
    }
    if ((execution_state_ == ugv_nav4d_ros2::msg::MissionStatus::PAUSED &&
         execution_can_resume_) ||
        execution_state_ == ugv_nav4d_ros2::msg::MissionStatus::FAILED ||
        execution_state_ == ugv_nav4d_ros2::msg::MissionStatus::ABORTED ||
        execution_state_ == ugv_nav4d_ros2::msg::MissionStatus::COMPLETED)
    {
        requestNativeRecovery();
        return;
    }
    if (attempts_remaining <= 0)
    {
        finishRecovery("Recovery stopped: controller did not confirm pause within 5 seconds; use STOP.");
        return;
    }
    QTimer::singleShot(250, this,
        [this, attempts_remaining]() { waitForRecoveryStop(attempts_remaining - 1); });
}

void OperatorPanel::requestNativeRecovery()
{
    if (!recovery_in_progress_ || !native_recovery_client_ ||
        !native_recovery_client_->service_is_ready())
    {
        finishRecovery("Recovery failed: native recovery service became unavailable.");
        return;
    }
    setStatusText("Recovery: finding a native out-of-obstacle trajectory...");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    native_recovery_client_->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
        {
            const auto response = future.get();
            QMetaObject::invokeMethod(this, [this, success = response->success,
                                                   message = response->message]()
            {
                const QString detail = QString::fromStdString(message);
                finishRecovery(success
                    ? "Native recovery preview ready: " + detail
                    : "Native recovery failed: " + detail);
            }, Qt::QueuedConnection);
        });
}

void OperatorPanel::finishRecovery(const QString& status)
{
    recovery_in_progress_ = false;
    if (recover_button_)
    {
        recover_button_->setEnabled(true);
    }
    setStatusText(status);
}

void OperatorPanel::onReplanMission()
{
    callTrigger(replan_mission_client_, "Replan");
}

void OperatorPanel::onExecutePath()
{
    callTrigger(execute_path_client_, "Execute path");
}

void OperatorPanel::onDiscardPath()
{
    callTrigger(discard_path_client_, "Discard path");
}

void OperatorPanel::onDeleteWaypoint()
{
    if (!edit_waypoint_client_ || !node_)
    {
        return;
    }
    if (!edit_waypoint_client_->service_is_ready())
    {
        setStatusText("Delete waypoint: service unavailable (is the planner node running?)");
        return;
    }
    auto request = std::make_shared<ugv_nav4d_ros2::srv::EditWaypoint::Request>();
    request->index = static_cast<uint32_t>(waypoint_index_spin_->value());
    request->remove = true;
    edit_waypoint_client_->async_send_request(request,
        [this](rclcpp::Client<ugv_nav4d_ros2::srv::EditWaypoint>::SharedFuture future)
        {
            setStatusText(QString::fromStdString(future.get()->message));
        });
}

void OperatorPanel::onDeleteZone()
{
    if (!delete_zone_client_ || !node_)
    {
        return;
    }
    if (!delete_zone_client_->service_is_ready())
    {
        setStatusText("Delete zone: service unavailable (is the planner node running?)");
        return;
    }
    auto request = std::make_shared<ugv_nav4d_ros2::srv::DeleteForbiddenZone::Request>();
    request->index = static_cast<uint32_t>(zone_index_spin_->value());
    delete_zone_client_->async_send_request(request,
        [this](rclcpp::Client<ugv_nav4d_ros2::srv::DeleteForbiddenZone>::SharedFuture future)
        {
            setStatusText(QString::fromStdString(future.get()->message));
        });
}

void OperatorPanel::onPlanReturn()
{
    callTrigger(plan_return_client_, "Plan return");
}

void OperatorPanel::onReturnModeChanged(int index)
{
    if (!set_return_forward_client_ || !node_ || !set_return_forward_client_->service_is_ready())
    {
        setStatusText("Return mode: service unavailable (is the planner node running?)");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = (index == 0);
    set_return_forward_client_->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
        {
            setStatusText(QString::fromStdString(future.get()->message));
        });
}

void OperatorPanel::onClearWaypoints()
{
    callTrigger(clear_waypoints_client_, "Clear waypoints");
}

void OperatorPanel::onUndoWaypoint()
{
    callTrigger(remove_last_waypoint_client_, "Undo waypoint");
}

void OperatorPanel::onClearZones()
{
    callTrigger(clear_zones_client_, "Clear zones");
}

void OperatorPanel::onUndoZone()
{
    callTrigger(undo_zone_client_, "Undo zone");
}

void OperatorPanel::onSaveMap()
{
    callTrigger(save_map_client_, "Save map");
}

void OperatorPanel::onRepublishMaps()
{
    callTrigger(map_publish_client_, "Republish maps");
}

void OperatorPanel::onRegenerateMaps()
{
    callTrigger(regenerate_maps_client_, "Regenerate maps");
}

void OperatorPanel::onSaveMission()
{
    callTrigger(save_mission_client_, "Save mission");
}

void OperatorPanel::onLoadMission()
{
    callTrigger(load_mission_client_, "Load mission");
}

} // namespace ugv_nav4d_ros2_operator_panel
} // namespace ugv_nav4d_ros2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ugv_nav4d_ros2::ugv_nav4d_ros2_operator_panel::OperatorPanel, rviz_common::Panel)
