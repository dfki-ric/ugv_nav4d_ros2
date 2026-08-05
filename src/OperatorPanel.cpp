#include "OperatorPanel.hpp"

#include <algorithm>

#include <QCheckBox>
#include <QGridLayout>
#include <QFileDialog>
#include <QFileInfo>
#include <QComboBox>
#include <QPair>
#include <QScrollArea>
#include <QVector>
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

    // Safety-critical control: created here but added to the OUTER layout at
    // the bottom of this constructor, pinned above the scroll area so it can
    // never be scrolled out of reach.
    stop_button_ = new QPushButton("STOP");
    stop_button_->setMinimumHeight(44);
    stop_button_->setStyleSheet("QPushButton { font-weight: bold; font-size: 16px; background-color: #b71c1c; color: white; padding: 6px; }");

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

    // One checkbox per readiness item (created lazily from SystemHealth
    // messages): only CHECKED items may block the Execute button, so the
    // operator can exempt e.g. flaky diagnostics without editing code.
    checks_group_ = new QGroupBox("Execute gating checks");
    checks_group_->setToolTip(
        "Only checked readiness items can block Execute. Unchecked items are "
        "still shown in the readiness list but ignored for gating.");
    checks_layout_ = new QGridLayout;
    checks_group_->setLayout(checks_layout_);

    // ros2_control controllers of /arter/ros_control/controller_manager:
    // checked = active. Toggling a box activates/deactivates that controller
    // (BEST_EFFORT), replacing the switch_controllers shell script for the
    // common field cases.
    controllers_group_ = new QGroupBox("ros2_control controllers");
    auto* controllers_box = new QVBoxLayout;
    controllers_status_label_ = new QLabel("(waiting for controller_manager)");
    controllers_status_label_->setWordWrap(true);
    controllers_box->addWidget(controllers_status_label_);
    controllers_layout_ = new QGridLayout;
    controllers_box->addLayout(controllers_layout_);
    auto* controllers_buttons = new QHBoxLayout;
    refresh_controllers_button_ = new QPushButton("Refresh");
    nav_controllers_button_ = new QPushButton("Enable navigation set");
    nav_controllers_button_->setToolTip(
        "Activate steer_position_controller, wheel_velocity_controller and "
        "steering_mode_controller in one switch.");
    controllers_buttons->addWidget(refresh_controllers_button_);
    controllers_buttons->addWidget(nav_controllers_button_);
    controllers_box->addLayout(controllers_buttons);
    controllers_group_->setLayout(controllers_box);

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
    // The unconditional green made the button look actionable even while
    // setEnabled(false); the :disabled state must override the background.
    execute_path_button_->setStyleSheet(
        "QPushButton { font-weight: bold; background-color: #2e7d32; color: white; }"
        "QPushButton:disabled { background-color: #55605a; color: #b0b0b0; }");
    discard_path_button_ = new QPushButton("Discard path");
    clear_waypoints_button_ = new QPushButton("Clear waypoints");
    undo_waypoint_button_ = new QPushButton("Undo last waypoint");
    reverse_waypoints_button_ = new QPushButton("Flip route direction");
    reverse_waypoints_button_->setToolTip(
        "Reverse the waypoint order and flip each waypoint heading by 180 deg,\n"
        "so the queued route is driven in the opposite direction. Set a goal and plan afterwards.");
    clear_zones_button_ = new QPushButton("Clear zones");
    undo_zone_button_ = new QPushButton("Undo last zone");
    save_map_button_ = new QPushButton("Save MLS map");
    republish_maps_button_ = new QPushButton("Republish maps");
    regenerate_maps_button_ = new QPushButton("Regenerate maps");
    update_footprint_button_ = new QPushButton("Update footprint");
    update_footprint_button_->setToolTip(
        "Measure the current wheel positions via TF (the wheelbase is variable) "
        "and apply the resulting footprint to the planner. Triggers a trav-map "
        "and planner rebuild when the size changed.");
    buttons->addWidget(pause_button_, 0, 0);
    buttons->addWidget(resume_button_, 0, 1);
    buttons->addWidget(recover_button_, 1, 0, 1, 2);
    buttons->addWidget(replan_button_, 2, 0, 1, 2);
    buttons->addWidget(execute_path_button_, 3, 0);
    buttons->addWidget(discard_path_button_, 3, 1);
    buttons->addWidget(clear_waypoints_button_, 4, 0);
    buttons->addWidget(undo_waypoint_button_, 4, 1);
    buttons->addWidget(reverse_waypoints_button_, 5, 0, 1, 2);
    buttons->addWidget(clear_zones_button_, 6, 0);
    buttons->addWidget(undo_zone_button_, 6, 1);
    buttons->addWidget(save_map_button_, 7, 0);
    buttons->addWidget(republish_maps_button_, 7, 1);
    buttons->addWidget(regenerate_maps_button_, 8, 0);
    buttons->addWidget(update_footprint_button_, 8, 1);
    save_mission_button_ = new QPushButton("Save mission");
    load_mission_button_ = new QPushButton("Load mission");
    buttons->addWidget(save_mission_button_, 9, 0);
    buttons->addWidget(load_mission_button_, 9, 1);
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

    // Configuration-style groups (rarely touched mid-mission) sit below all
    // mission controls, at the end of the scrollable content.
    layout->addWidget(checks_group_);
    layout->addWidget(controllers_group_);

    layout->addStretch();

    // The content outgrew typical screen heights, which also stopped RViz
    // from shrinking the panel (a panel's minimum size bounds the window).
    // Everything except STOP lives in a scroll area, so the panel can be
    // resized down to roughly the STOP button alone.
    auto* content = new QWidget;
    content->setLayout(layout);
    auto* scroll = new QScrollArea;
    scroll->setWidget(content);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    auto* outer = new QVBoxLayout;
    outer->setContentsMargins(0, 0, 0, 0);
    outer->addWidget(stop_button_);
    outer->addWidget(scroll);
    setLayout(outer);

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
    connect(reverse_waypoints_button_, &QPushButton::clicked, this, &OperatorPanel::onReverseWaypoints);
    connect(clear_zones_button_, &QPushButton::clicked, this, &OperatorPanel::onClearZones);
    connect(undo_zone_button_, &QPushButton::clicked, this, &OperatorPanel::onUndoZone);
    connect(save_map_button_, &QPushButton::clicked, this, &OperatorPanel::onSaveMap);
    connect(republish_maps_button_, &QPushButton::clicked, this, &OperatorPanel::onRepublishMaps);
    connect(regenerate_maps_button_, &QPushButton::clicked, this, &OperatorPanel::onRegenerateMaps);
    connect(update_footprint_button_, &QPushButton::clicked, this, [this]() {
        callTrigger(update_footprint_client_, "Update footprint");
    });
    connect(save_mission_button_, &QPushButton::clicked, this, &OperatorPanel::onSaveMission);
    connect(load_mission_button_, &QPushButton::clicked, this, &OperatorPanel::onLoadMission);
    connect(refresh_controllers_button_, &QPushButton::clicked, this,
            &OperatorPanel::refreshControllers);
    connect(nav_controllers_button_, &QPushButton::clicked, this, [this]() {
        switchControllers({"steer_position_controller", "wheel_velocity_controller",
                           "steering_mode_controller"},
                          {}, "Enable navigation controllers");
    });
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
        "/ugv_nav4d_ros2/execution_status", rclcpp::QoS(1).transient_local(),
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
                updateExecuteEnabled();
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
            QVector<ReadinessItem> items;
            const size_t count = std::min({
                msg->readiness_names.size(),
                msg->readiness_levels.size(),
                msg->readiness_messages.size()});
            for (size_t i = 0; i < count; ++i) {
                items.append({QString::fromStdString(msg->readiness_names[i]),
                              msg->readiness_levels[i],
                              QString::fromStdString(msg->readiness_messages[i])});
            }
            QMetaObject::invokeMethod(this, [this, text, items]() {
                // The label itself is rebuilt from the CHECKED readiness items
                // in rebuildReadinessLabel(); the unfiltered node-side summary
                // is kept for the tooltip only.
                health_node_summary_ = text;
                for (const auto& item : items) {
                    ensureReadinessCheckbox(item.name);
                }
                readiness_items_ = items;
                health_received_ = true;
                rebuildReadinessLabel();
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
    reverse_waypoints_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/reverse_waypoints");
    clear_zones_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/clear_forbidden_zones");
    undo_zone_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/remove_last_forbidden_zone");
    save_map_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/save_mls_map");
    map_publish_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/map_publish");
    regenerate_maps_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/regenerate_maps");
    update_footprint_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/update_footprint");
    save_mission_client_ = node_->create_client<ugv_nav4d_ros2::srv::MissionFile>("/ugv_nav4d_ros2/save_mission");
    load_mission_client_ = node_->create_client<ugv_nav4d_ros2::srv::MissionFile>("/ugv_nav4d_ros2/load_mission");

    list_controllers_client_ = node_->create_client<controller_manager_msgs::srv::ListControllers>(
        "/arter/ros_control/controller_manager/list_controllers");
    switch_controller_client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
        "/arter/ros_control/controller_manager/switch_controller");
    auto* controllers_timer = new QTimer(this);
    connect(controllers_timer, &QTimer::timeout, this, &OperatorPanel::refreshControllers);
    controllers_timer->start(3000);
    refreshControllers();
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

QCheckBox* OperatorPanel::ensureReadinessCheckbox(const QString& name)
{
    const auto existing = readiness_checks_.find(name);
    if (existing != readiness_checks_.end())
    {
        return existing.value();
    }
    auto* box = new QCheckBox(name);
    box->setChecked(saved_check_states_.value(name, true));
    box->setToolTip(QString("Checked: an ERROR on \"%1\" blocks Execute. "
                            "Unchecked: this check is ignored for gating.").arg(name));
    const int position = readiness_checks_.size();
    checks_layout_->addWidget(box, position / 2, position % 2);
    readiness_checks_.insert(name, box);
    connect(box, &QCheckBox::toggled, this, [this](bool) {
        rebuildReadinessLabel();
        updateExecuteEnabled();
        Q_EMIT configChanged();
    });
    return box;
}

bool OperatorPanel::checkedReadinessOk(QString* blocking_check) const
{
    for (const auto& item : readiness_items_)
    {
        if (item.level != ugv_nav4d_ros2::msg::SystemHealth::ERROR)
        {
            continue;
        }
        const auto box = readiness_checks_.constFind(item.name);
        if (box != readiness_checks_.cend() && !box.value()->isChecked())
        {
            continue;
        }
        if (blocking_check)
        {
            *blocking_check = item.name;
        }
        return false;
    }
    return true;
}

void OperatorPanel::rebuildReadinessLabel()
{
    if (!health_received_)
    {
        readiness_label_->setText("Readiness: waiting");
        readiness_label_->setToolTip("Waiting for the first system-health report.");
        readiness_label_->setStyleSheet("QLabel { font-family: monospace; }");
        return;
    }
    QStringList rows;
    QStringList tooltip_rows;
    QStringList entries;
    bool any_error = false;
    bool any_warn = false;
    int ignored = 0;
    for (const auto& item : readiness_items_)
    {
        const auto box = readiness_checks_.constFind(item.name);
        if (box != readiness_checks_.cend() && !box.value()->isChecked())
        {
            ++ignored;
            continue;
        }
        const QString badge =
            item.level == ugv_nav4d_ros2::msg::SystemHealth::OK ? "OK  " :
            item.level == ugv_nav4d_ros2::msg::SystemHealth::WARN ? "WARN" : "ERR ";
        rows << QString("%1  %2").arg(badge, item.name);
        tooltip_rows << QString("%1  %2: %3").arg(badge, item.name, item.message);
        // Every checked item is listed, healthy or not; unchecked ones are
        // omitted entirely.
        entries << QString("%1: %2").arg(item.name, item.message);
        any_error = any_error || item.level == ugv_nav4d_ros2::msg::SystemHealth::ERROR;
        any_warn = any_warn || item.level == ugv_nav4d_ros2::msg::SystemHealth::WARN;
    }
    health_label_->setText(entries.isEmpty()
        ? QString("System health: no checks selected")
        : "System health: " + entries.join("; "));
    health_label_->setToolTip("All checks (node-side): " + health_node_summary_);
    health_label_->setStyleSheet(any_error
        ? "QLabel { color: #c62828; font-weight: bold; }"
        : any_warn
            ? "QLabel { color: #ef6c00; }"
            : "QLabel { color: #2e7d32; }");
    // "ready" mirrors the health part of the Execute gate: no ERROR among the
    // CHECKED items. Unchecked items are hidden here; the gating-checks group
    // below remains the full inventory.
    const bool ready = !any_error;
    QString header = QString("Readiness: %1").arg(ready ? "ready" : "not ready");
    if (ignored > 0)
    {
        header += QString(" (%1 unchecked hidden)").arg(ignored);
    }
    const QString text = rows.isEmpty() ? header : header + '\n' + rows.join('\n');
    readiness_label_->setText(text);
    readiness_label_->setToolTip(
        tooltip_rows.isEmpty() ? text : header + '\n' + tooltip_rows.join('\n'));
    readiness_label_->setStyleSheet(ready
        ? "QLabel { color: #2e7d32; font-family: monospace; }"
        : "QLabel { color: #c62828; font-family: monospace; }");
}

void OperatorPanel::refreshControllers()
{
    if (!list_controllers_client_)
    {
        return;
    }
    if (!list_controllers_client_->service_is_ready())
    {
        controllers_status_label_->setText(
            "controller_manager unavailable (/arter/ros_control/controller_manager)");
        for (auto it = controller_checks_.cbegin(); it != controller_checks_.cend(); ++it)
        {
            it.value()->setEnabled(false);
        }
        return;
    }
    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    list_controllers_client_->async_send_request(request,
        [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future)
        {
            const auto& response = future.get();
            QVector<QPair<QString, QString>> states;
            for (const auto& controller : response->controller)
            {
                states.append({QString::fromStdString(controller.name),
                               QString::fromStdString(controller.state)});
            }
            QMetaObject::invokeMethod(this, [this, states]() {
                applyControllerStates(states);
            }, Qt::QueuedConnection);
        });
}

void OperatorPanel::applyControllerStates(const QVector<QPair<QString, QString>>& states)
{
    int active = 0;
    for (const auto& state : states)
    {
        auto it = controller_checks_.find(state.first);
        if (it == controller_checks_.end())
        {
            auto* box = new QCheckBox;
            const int position = controller_checks_.size();
            controllers_layout_->addWidget(box, position / 2, position % 2);
            it = controller_checks_.insert(state.first, box);
            connect(box, &QCheckBox::clicked, this, [this, name = state.first](bool checked) {
                switchControllers(checked ? QStringList{name} : QStringList{},
                                  checked ? QStringList{} : QStringList{name},
                                  (checked ? "Activate " : "Deactivate ") + name);
            });
        }
        QCheckBox* box = it.value();
        // clicked (not toggled) drives the switch, so programmatic state
        // updates here cannot echo back into switch_controller requests.
        box->setChecked(state.second == "active");
        box->setText(QString("%1 [%2]").arg(state.first, state.second));
        // Activation requires the controller to be at least configured;
        // unconfigured/finalized ones need `ros2 control` on the console.
        box->setEnabled(state.second == "active" || state.second == "inactive");
        box->setToolTip(state.second == "unconfigured"
            ? "Unconfigured: load/configure it first (ros2 control set_controller_state)."
            : "Checked = active. Toggle to activate/deactivate via controller_manager.");
        if (state.second == "active")
        {
            ++active;
        }
    }
    controllers_status_label_->setText(
        QString("%1 controller%2, %3 active")
            .arg(states.size()).arg(states.size() == 1 ? "" : "s").arg(active));
}

void OperatorPanel::switchControllers(const QStringList& activate, const QStringList& deactivate,
                                      const QString& actionName)
{
    if (!switch_controller_client_ || !switch_controller_client_->service_is_ready())
    {
        setStatusText(actionName + ": controller_manager unavailable.");
        refreshControllers();
        return;
    }
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    for (const auto& name : activate)
    {
        request->activate_controllers.push_back(name.toStdString());
    }
    for (const auto& name : deactivate)
    {
        request->deactivate_controllers.push_back(name.toStdString());
    }
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    setStatusText(actionName + " requested...");
    switch_controller_client_->async_send_request(request,
        [this, actionName](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future)
        {
            const bool ok = future.get()->ok;
            setStatusText(actionName + (ok ? " done." : " failed (see controller_manager log)."));
            QMetaObject::invokeMethod(this, [this]() { refreshControllers(); },
                                      Qt::QueuedConnection);
        });
}

void OperatorPanel::save(rviz_common::Config config) const
{
    rviz_common::Panel::save(config);
    rviz_common::Config checks = config.mapMakeChild("gating_checks");
    for (auto it = readiness_checks_.cbegin(); it != readiness_checks_.cend(); ++it)
    {
        checks.mapSetValue(it.key(), it.value()->isChecked());
    }
}

void OperatorPanel::load(const rviz_common::Config& config)
{
    rviz_common::Panel::load(config);
    const rviz_common::Config checks = config.mapGetChild("gating_checks");
    if (!checks.isValid())
    {
        return;
    }
    for (auto it = checks.mapIterator(); it.isValid(); it.advance())
    {
        saved_check_states_.insert(it.currentKey(), it.currentChild().getValue().toBool());
    }
    for (auto it = readiness_checks_.cbegin(); it != readiness_checks_.cend(); ++it)
    {
        it.value()->setChecked(saved_check_states_.value(it.key(), it.value()->isChecked()));
    }
    rebuildReadinessLabel();
    updateExecuteEnabled();
}

void OperatorPanel::updateExecuteEnabled()
{
    const bool executing =
        execution_state_ == ugv_nav4d_ros2::msg::MissionStatus::EXECUTING;
    const bool paused =
        execution_state_ == ugv_nav4d_ros2::msg::MissionStatus::PAUSED;

    // Execute stays available while PAUSED: the replan/recovery flows produce
    // a new pending path that the operator confirms with Execute.
    QString blocking_check;
    const bool health_ok = health_received_ && checkedReadinessOk(&blocking_check);
    const bool execute_ok = health_ok && route_ready_ && !executing;
    execute_path_button_->setEnabled(execute_ok);
    execute_path_button_->setToolTip(
        executing
            ? "A mission is already executing; pause or stop it first."
            : execute_ok
                ? "Send the reviewed path to the controller."
                : !health_received_
                    ? "Waiting for the first system-health report."
                    : !blocking_check.isEmpty()
                        ? QString("Blocked by readiness check \"%1\" — fix it or "
                                  "uncheck it under Execute gating checks.").arg(blocking_check)
                        : "Execution requires an approved route preview.");

    pause_button_->setEnabled(executing);
    pause_button_->setToolTip(executing
        ? "Pause the running mission; the remaining route is kept."
        : "Nothing is executing.");
    resume_button_->setEnabled(paused && execution_can_resume_);
    resume_button_->setToolTip(
        paused && execution_can_resume_
            ? "Continue the paused mission from the robot's position."
            : paused
                ? "Waiting for the controller to confirm the pause..."
                : "No paused mission.");
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

void OperatorPanel::onReverseWaypoints()
{
    callTrigger(reverse_waypoints_client_, "Flip route direction");
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

void OperatorPanel::callMissionFile(const rclcpp::Client<ugv_nav4d_ros2::srv::MissionFile>::SharedPtr& client,
                                    const QString& filename, const QString& actionName)
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
    auto request = std::make_shared<ugv_nav4d_ros2::srv::MissionFile::Request>();
    request->filename = filename.toStdString();
    client->async_send_request(request,
        [this, actionName](rclcpp::Client<ugv_nav4d_ros2::srv::MissionFile>::SharedFuture future)
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

void OperatorPanel::onSaveMission()
{
    // File dialog runs on the robot-side rviz host; the chosen FULL path is sent
    // to the planner node, which writes the file.
    QString file = QFileDialog::getSaveFileName(
        this, "Save mission (waypoints + zones)",
        last_mission_dir_.isEmpty() ? QString("ugv_nav4d_mission.txt")
                                    : last_mission_dir_ + "/ugv_nav4d_mission.txt",
        "Mission files (*.txt);;All files (*)");
    if (file.isEmpty())
    {
        return; // dialog cancelled
    }
    last_mission_dir_ = QFileInfo(file).absolutePath();
    callMissionFile(save_mission_client_, file, "Save mission");
}

void OperatorPanel::onLoadMission()
{
    QString file = QFileDialog::getOpenFileName(
        this, "Load mission (waypoints + zones)", last_mission_dir_,
        "Mission files (*.txt);;All files (*)");
    if (file.isEmpty())
    {
        return; // dialog cancelled
    }
    last_mission_dir_ = QFileInfo(file).absolutePath();
    callMissionFile(load_mission_client_, file, "Load mission");
}

} // namespace ugv_nav4d_ros2_operator_panel
} // namespace ugv_nav4d_ros2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ugv_nav4d_ros2::ugv_nav4d_ros2_operator_panel::OperatorPanel, rviz_common::Panel)
