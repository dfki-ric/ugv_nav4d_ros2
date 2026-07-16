#include "OperatorPanel.hpp"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
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

    status_label_ = new QLabel("(waiting for planner status)");
    status_label_->setWordWrap(true);
    status_label_->setStyleSheet("QLabel { font-weight: bold; }");
    layout->addWidget(new QLabel("Planner status:"));
    layout->addWidget(status_label_);

    auto* buttons = new QGridLayout;
    execute_path_button_ = new QPushButton("Execute path");
    execute_path_button_->setStyleSheet("QPushButton { font-weight: bold; background-color: #2e7d32; color: white; }");
    discard_path_button_ = new QPushButton("Discard path");
    clear_waypoints_button_ = new QPushButton("Clear waypoints");
    undo_waypoint_button_ = new QPushButton("Undo last waypoint");
    clear_zones_button_ = new QPushButton("Clear zones");
    undo_zone_button_ = new QPushButton("Undo last zone");
    save_map_button_ = new QPushButton("Save MLS map");
    republish_maps_button_ = new QPushButton("Republish maps");
    buttons->addWidget(execute_path_button_, 0, 0);
    buttons->addWidget(discard_path_button_, 0, 1);
    buttons->addWidget(clear_waypoints_button_, 1, 0);
    buttons->addWidget(undo_waypoint_button_, 1, 1);
    buttons->addWidget(clear_zones_button_, 2, 0);
    buttons->addWidget(undo_zone_button_, 2, 1);
    buttons->addWidget(save_map_button_, 3, 0);
    buttons->addWidget(republish_maps_button_, 3, 1);
    layout->addLayout(buttons);

    auto* wp_row = new QHBoxLayout;
    wp_row->addWidget(new QLabel("WP #"));
    waypoint_index_spin_ = new QSpinBox;
    waypoint_index_spin_->setMinimum(1);
    waypoint_index_spin_->setMaximum(999);
    wp_row->addWidget(waypoint_index_spin_);
    delete_waypoint_button_ = new QPushButton("Delete waypoint");
    wp_row->addWidget(delete_waypoint_button_);
    layout->addLayout(wp_row);

    layout->addStretch();
    setLayout(layout);

    connect(execute_path_button_, &QPushButton::clicked, this, &OperatorPanel::onExecutePath);
    connect(discard_path_button_, &QPushButton::clicked, this, &OperatorPanel::onDiscardPath);
    connect(delete_waypoint_button_, &QPushButton::clicked, this, &OperatorPanel::onDeleteWaypoint);
    connect(clear_waypoints_button_, &QPushButton::clicked, this, &OperatorPanel::onClearWaypoints);
    connect(undo_waypoint_button_, &QPushButton::clicked, this, &OperatorPanel::onUndoWaypoint);
    connect(clear_zones_button_, &QPushButton::clicked, this, &OperatorPanel::onClearZones);
    connect(undo_zone_button_, &QPushButton::clicked, this, &OperatorPanel::onUndoZone);
    connect(save_map_button_, &QPushButton::clicked, this, &OperatorPanel::onSaveMap);
    connect(republish_maps_button_, &QPushButton::clicked, this, &OperatorPanel::onRepublishMaps);
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

    execute_path_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/execute_path");
    discard_path_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/discard_path");
    edit_waypoint_client_ = node_->create_client<ugv_nav4d_ros2::srv::EditWaypoint>("/ugv_nav4d_ros2/edit_waypoint");
    clear_waypoints_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/clear_waypoints");
    remove_last_waypoint_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/remove_last_waypoint");
    clear_zones_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/clear_forbidden_zones");
    undo_zone_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/remove_last_forbidden_zone");
    save_map_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/save_mls_map");
    map_publish_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/map_publish");
}

void OperatorPanel::setStatusText(const QString& text)
{
    // Subscription and service callbacks run on the ROS spin thread; Qt widgets
    // must only be touched from the GUI thread.
    QMetaObject::invokeMethod(status_label_, [label = status_label_, text]()
    {
        label->setText(text);
    }, Qt::QueuedConnection);
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

} // namespace ugv_nav4d_ros2_operator_panel
} // namespace ugv_nav4d_ros2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ugv_nav4d_ros2::ugv_nav4d_ros2_operator_panel::OperatorPanel, rviz_common::Panel)
