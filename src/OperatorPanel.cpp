#include "OperatorPanel.hpp"
#include <rclcpp/parameter_client.hpp>
#include <QDoubleSpinBox>

#include <algorithm>

#include <QCheckBox>
#include <QGridLayout>
#include <QFileDialog>
#include <QPointer>
#include <QFileInfo>
#include <QComboBox>
#include <QPair>
#include <QScrollArea>
#include <QVector>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <cstdio>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
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

//Drive controllers the follower needs; activating an already-active
//controller is a BEST_EFFORT no-op, so this set is safe to (re)apply.
static const QStringList kNavigationControllers = {
    "steer_position_controller", "wheel_velocity_controller", "steering_mode_controller"};

OperatorPanel::OperatorPanel(QWidget* parent)
: rviz_common::Panel(parent)
{
    // Field usability: every button a comfortable click target (small screens,
    // gloves, sunlight). Panel-wide rule so new buttons inherit it; buttons
    // with their own stylesheet (Abort/Pause/Resume) still win on conflicts.
    // Checkbox/spinbox hit areas are bumped for the same reason.
    setStyleSheet(
        "QPushButton { min-height: 34px; padding: 4px 10px; }"
        "QCheckBox { min-height: 30px; spacing: 8px; }"
        "QCheckBox::indicator { width: 26px; height: 26px; }"
        "QSpinBox, QDoubleSpinBox { min-height: 38px; font-size: 14px;"
        " padding: 2px 6px; }"
        "QSpinBox::up-button, QSpinBox::down-button,"
        "QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {"
        " width: 24px; }");

    auto* layout = new QVBoxLayout;

    // Deliberate mission-ending control. Named for what it does: it cancels
    // execution AND drops the retained mission (not resumable) — the quick
    // "make the robot hold" control is Pause, pinned at the top. Sized like
    // its row neighbours and placed next to Execute; red conveys the danger.
    stop_button_ = new QPushButton("Abort mission");
    stop_button_->setStyleSheet(
        "QPushButton { font-weight: bold; background-color: #b71c1c; color: white; }");
    stop_button_->setToolTip(
        "Cancel execution and DROP the mission (Resume will not work).\n"
        "For a resumable stop use Pause.");

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
    deviation_label_ = new QLabel("Deviation: idle");
    deviation_label_->setWordWrap(true);
    deviation_label_->setMaximumHeight(40);
    footprint_label_ = new QLabel("Footprint: press Update footprint to measure");
    footprint_label_->setWordWrap(true);
    footprint_label_->setMaximumHeight(40);
    height_label_ = new QLabel("Height: waiting for planner");
    height_label_->setWordWrap(true);
    height_label_->setMaximumHeight(40);
    cmd_vel_label_ = new QLabel("cmd_vel: (none)");
    cmd_vel_label_->setWordWrap(true);
    cmd_vel_label_->setMaximumHeight(40);
    bag_label_ = new QLabel("Bag: recorder not seen");
    bag_label_->setWordWrap(true);
    bag_label_->setMaximumHeight(40);
    inspection_label_ = new QLabel("Inspection: select Inspect Traversability and click a map patch");
    inspection_label_->setWordWrap(true);
    inspection_label_->setMaximumHeight(60);
    inspection_label_->setStyleSheet("QLabel { color: #69a8ff; }");
    layout->addWidget(execution_label_);
    layout->addWidget(risk_label_);
    layout->addWidget(deviation_label_);
    layout->addWidget(footprint_label_);
    layout->addWidget(height_label_);
    layout->addWidget(cmd_vel_label_);
    layout->addWidget(bag_label_);
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
    controllers_status_label_ = new QLabel("(press Refresh to query controllers)");
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
    resume_button_ = new QPushButton("Resume current path");
    recover_button_ = new QPushButton("Recover from obstacle");
    recover_button_->setStyleSheet(
        "QPushButton { font-weight: bold; background-color: #ef6c00; color: white; padding: 6px; }");
    recover_button_->setToolTip(
        "Safely pause active execution, then use ugv_nav4d's native out-of-obstacle recovery to create an approval-gated rescue trajectory.");
    replan_button_ = new QPushButton("Replan from robot");
    execute_path_button_ = new QPushButton("Start new path");
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
    recalibrate_height_button_ = new QPushButton("Recalibrate height");
    recalibrate_height_button_->setToolTip(
        "Measure the ground patch under the robot and set distToGround = pose.z - patch.z.\n"
        "Use after the chassis changed its height (adaptation), when planning fails with\n"
        "START_INVALID, or when the height discs in the 3D view separate.");
    update_footprint_button_ = new QPushButton("Update footprint");
    update_footprint_button_->setToolTip(
        "Measure the current wheel positions via TF (the wheelbase is variable) "
        "and apply the resulting footprint to the planner. Triggers a trav-map "
        "and planner rebuild when the size changed.");
    pause_at_wp_check_ = new QCheckBox("Pause at each waypoint (manual resume)");
    pause_at_wp_check_->setEnabled(false);
    pause_at_wp_check_->setToolTip(
        "When enabled, execution pauses once near every queued waypoint and only\n"
        "continues after Resume. Needs at least one waypoint in the queue; a plain\n"
        "single-goal route has none, so the toggle stays disabled.");
    auto_plan_check_ = new QCheckBox("Auto plan");
    auto_plan_check_->setToolTip(
        "While enabled, every waypoint change replans the whole chain as a\n"
        "preview: the last waypoint acts as the goal, earlier ones are visited\n"
        "in order. Execution still requires the Execute button. Works while a\n"
        "mission is executing (preview only).");
    buttons->addWidget(pause_at_wp_check_, 1, 0, 1, 1);
    buttons->addWidget(auto_plan_check_, 1, 1, 1, 1);
    // Pause/Resume are NOT in this grid: they live pinned next to STOP at the
    // top of the panel, so they stay reachable without scrolling on small
    // screens (operator request: scrolling mid-mission causes misclicks).
    buttons->addWidget(recover_button_, 2, 0, 1, 2);
    plan_from_button_ = new QPushButton("Plan from WP");
    plan_from_button_->setToolTip(
        "Drop every waypoint before WP # (the box to the left), then plan from\n"
        "the current robot pose through WP # and the remaining waypoints.\n"
        "Produces a preview — confirm with Start new path.");
    buttons->addWidget(replan_button_, 3, 0, 1, 2);
    // Start (ex-Execute path) lives pinned under Resume at the top of the
    // panel; this grid row keeps the remaining mission-ending pair.
    buttons->addWidget(stop_button_, 4, 0);
    buttons->addWidget(discard_path_button_, 4, 1);
    buttons->addWidget(clear_waypoints_button_, 5, 0);
    buttons->addWidget(undo_waypoint_button_, 5, 1);
    buttons->addWidget(reverse_waypoints_button_, 6, 0, 1, 2);
    buttons->addWidget(clear_zones_button_, 7, 0);
    buttons->addWidget(undo_zone_button_, 7, 1);
    buttons->addWidget(save_map_button_, 8, 0);
    buttons->addWidget(republish_maps_button_, 8, 1);
    buttons->addWidget(regenerate_maps_button_, 9, 0);
    buttons->addWidget(update_footprint_button_, 9, 1);
    save_mission_button_ = new QPushButton("Save mission");
    load_mission_button_ = new QPushButton("Load mission");
    buttons->addWidget(save_mission_button_, 10, 0);
    buttons->addWidget(load_mission_button_, 10, 1);
    calibrate_geometry_button_ = new QPushButton("Calibrate geometry (footprint + height)");
    calibrate_geometry_button_->setToolTip(
        "One-click after-adaptation calibration: measures the footprint via TF\n"
        "(wheels + tool), recalibrates distToGround against the MLS under the\n"
        "robot, and triggers a SINGLE map+planner rebuild for both.");
    buttons->addWidget(calibrate_geometry_button_, 11, 0, 1, 2);
    buttons->addWidget(recalibrate_height_button_, 12, 0, 1, 2);
    record_bag_button_ = new QPushButton("\u25CF Record nav bag");
    record_bag_button_->setCheckable(true);
    record_bag_button_->setToolTip(
        "Start/stop a rosbag ON THE ROBOT with all navigation topics\n"
        "(planner + follower namespaces, TF, cmd_vel, odometries, rosout,\n"
        "input point cloud). Written to /opt/workspace/bags on the robot.");
    clear_executed_path_button_ = new QPushButton("Clear executed path");
    clear_executed_path_button_->setToolTip(
        "Remove the finished/stopped route from the 3D view (display only).\n"
        "Refused while a route is executing or paused.");
    buttons->addWidget(clear_executed_path_button_, 13, 0, 1, 2);
    buttons->addWidget(record_bag_button_, 14, 0, 1, 2);

    mls_delete_button_ = new QPushButton("Delete MLS patches (last Trav zone)");
    mls_delete_button_->setToolTip(
        "Deletes all MLS patches inside the last drawn 'Traversable fill (MLS edit)'\n"
        "zone, up to the ceiling. Pure MLS edit: planning is untouched until you\n"
        "click 'Regenerate trav map'. Draw the polygon with the zone tool first.");
    buttons->addWidget(mls_delete_button_, 15, 0, 1, 1);
    auto* del_row = new QHBoxLayout();
    delete_top_spin_ = new QDoubleSpinBox();
    delete_top_spin_->setRange(0.2, 10.0);
    delete_top_spin_->setSingleStep(0.1);
    delete_top_spin_->setDecimals(1);
    delete_top_spin_->setValue(2.0);
    delete_top_spin_->setSuffix(" m");
    delete_top_spin_->setToolTip(
        "Deletion ceiling: patches higher than this above the highest clicked\n"
        "vertex survive, so overhead structures beyond robot height stay in the\n"
        "map. Used by both Delete and Fill.");
    del_row->addWidget(new QLabel("up to"));
    del_row->addWidget(delete_top_spin_);
    buttons->addLayout(del_row, 15, 1, 1, 1);
    auto* fill_row = new QHBoxLayout();
    fill_auto_check_ = new QCheckBox("auto");
    fill_auto_check_->setChecked(true);
    fill_auto_check_->setToolTip(
        "Fit the fill plane to the ground: the planner least-squares fits a plane\n"
        "to the MLS ground in a ring just outside the polygon (height AND tilt),\n"
        "and z / roll / pitch become fine-trim offsets on top of that fit.\n"
        "Uncheck for the old fully manual plane.");
    fill_z_spin_ = new QDoubleSpinBox();
    fill_z_spin_->setRange(-5.0, 5.0);
    fill_z_spin_->setSingleStep(0.05);
    fill_z_spin_->setDecimals(2);
    fill_z_spin_->setSuffix(" m");
    fill_z_spin_->setToolTip(
        "Fill plane height relative to the mean z of the polygon's clicked vertices\n"
        "(clicks land on grass tops, so manual mode starts around -0.3).\n"
        "With 'auto' checked this is a trim on top of the fitted height; keep 0 first.");
    fill_roll_spin_ = new QDoubleSpinBox();
    fill_roll_spin_->setRange(-45.0, 45.0);
    fill_roll_spin_->setSingleStep(0.5);
    fill_roll_spin_->setDecimals(1);
    fill_roll_spin_->setSuffix(QString::fromUtf8("\u00B0"));
    fill_roll_spin_->setToolTip("Fill plane tilt about the world X axis.\nWith 'auto' checked this is a trim on top of the fitted tilt.");
    fill_pitch_spin_ = new QDoubleSpinBox();
    fill_pitch_spin_->setRange(-45.0, 45.0);
    fill_pitch_spin_->setSingleStep(0.5);
    fill_pitch_spin_->setDecimals(1);
    fill_pitch_spin_->setSuffix(QString::fromUtf8("\u00B0"));
    fill_pitch_spin_->setToolTip("Fill plane tilt about the world Y axis.\nWith 'auto' checked this is a trim on top of the fitted tilt.");
    fill_row->addWidget(fill_auto_check_);
    fill_row->addWidget(new QLabel("z"));
    fill_row->addWidget(fill_z_spin_);
    fill_row->addWidget(new QLabel("roll"));
    fill_row->addWidget(fill_roll_spin_);
    fill_row->addWidget(new QLabel("pitch"));
    fill_row->addWidget(fill_pitch_spin_);
    buttons->addLayout(fill_row, 16, 0, 1, 1);
    mls_fill_button_ = new QPushButton("Fill plane");
    mls_fill_button_->setToolTip(
        "Refills the last Traversable zone with synthetic flat ground. With 'auto'\n"
        "checked the plane is fitted to the surrounding ground (height + tilt) and\n"
        "z / roll / pitch trim the fit; unchecked they set the plane directly.\n"
        "Pure MLS edit: each click wipes the previous fill and replaces it, so\n"
        "iterate until it lines up with the ground, then click 'Regenerate trav\n"
        "map'. The fit persists in zones and missions.");
    buttons->addWidget(mls_fill_button_, 16, 1, 1, 1);
    regen_travmap_button_ = new QPushButton("Regenerate trav map (apply MLS edits)");
    regen_travmap_button_->setToolTip(
        "Applies ALL Traversable zone recipes (delete + fill) to the current\n"
        "MLS and rebuilds the traversability map + planner environment.\n"
        "This is the EDITED state. 'Regenerate maps' is the opposite: it\n"
        "reloads the original MLS from source with no edits applied.");
    buttons->addWidget(regen_travmap_button_, 17, 0, 1, 2);
    groom_check_ = new QCheckBox("Groom MLS under robot (wheels prove ground)");
    groom_check_->setToolTip(
        "While enabled, the MLS is flattened under the wheel footprint as the\n"
        "robot moves: ground = base_link minus the CALIBRATED distToGround\n"
        "(recalibrate height first!), patches in the band above it are deleted,\n"
        "and a plane tilted like the chassis is filled in. Pure MLS edit:\n"
        "'Regenerate trav map' applies it, 'Save map' keeps it, 'Regenerate\n"
        "maps' discards it.");
    buttons->addWidget(groom_check_, 18, 0, 1, 1);
    auto* groom_row = new QHBoxLayout();
    groom_top_spin_ = new QDoubleSpinBox();
    groom_top_spin_->setRange(0.2, 5.0);
    groom_top_spin_->setSingleStep(0.1);
    groom_top_spin_->setDecimals(1);
    groom_top_spin_->setValue(2.0);
    groom_top_spin_->setSuffix(" m");
    groom_top_spin_->setKeyboardTracking(false);
    groom_top_spin_->setToolTip(
        "Grooming ceiling (groom_delete_top): patches higher than this above\n"
        "the measured ground survive the under-wheel delete. Applied to the\n"
        "planner parameter when you finish editing; no rebuild involved.");
    groom_row->addWidget(new QLabel("del top"));
    groom_row->addWidget(groom_top_spin_);
    groom_margin_spin_ = new QDoubleSpinBox();
    groom_margin_spin_->setRange(0.0, 1.5);
    groom_margin_spin_->setSingleStep(0.05);
    groom_margin_spin_->setDecimals(2);
    groom_margin_spin_->setValue(0.2);
    groom_margin_spin_->setSuffix(" m");
    groom_margin_spin_->setKeyboardTracking(false);
    groom_margin_spin_->setToolTip(
        "Grooming margin (groom_margin) added uniformly around the measured\n"
        "wheel envelope; independent of the planner's footprint margins.\n"
        "Applied when you finish editing; no rebuild.");
    groom_row->addWidget(new QLabel("margin"));
    groom_row->addWidget(groom_margin_spin_);
    buttons->addLayout(groom_row, 18, 1, 1, 1);
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

    // The WP # spin box itself lives in the pinned top stack (next to
    // Plan from WP); these buttons read the same box.
    waypoint_index_spin_ = new QSpinBox;
    waypoint_index_spin_->setMinimum(1);
    waypoint_index_spin_->setMaximum(999);
    waypoint_index_spin_->setPrefix("WP ");
    waypoint_index_spin_->setToolTip(
        "Waypoint number used by Plan from WP and the Delete waypoint /\n"
        "Delete after / Delete before buttons.");
    auto* wp_row = new QHBoxLayout;
    delete_waypoint_button_ = new QPushButton("Delete waypoint");
    delete_waypoint_button_->setToolTip("Delete waypoint WP # (top bar).");
    wp_row->addWidget(delete_waypoint_button_);
    truncate_waypoints_button_ = new QPushButton("Delete after");
    truncate_waypoints_button_->setToolTip(
        "Remove every waypoint after WP # (top bar; keeps waypoints 1..#).");
    wp_row->addWidget(truncate_waypoints_button_);
    truncate_before_button_ = new QPushButton("Delete before");
    truncate_before_button_->setToolTip(
        "Remove every waypoint before WP # (top bar; keeps # and later; "
        "remaining renumber from 1).");
    wp_row->addWidget(truncate_before_button_);
    wp_row->addWidget(new QLabel("Zone #"));
    zone_index_spin_ = new QSpinBox;
    zone_index_spin_->setMinimum(1);
    zone_index_spin_->setMaximum(999);
    wp_row->addWidget(zone_index_spin_);
    delete_zone_button_ = new QPushButton("Delete zone");
    wp_row->addWidget(delete_zone_button_);
    layout->addLayout(wp_row);

    // Assigned only now that every listed widget exists: entries added to this
    // list before their `new` would be frozen as nullptr and silently skip the
    // rebuild lock-out.
    rebuild_sensitive_buttons_ = {
        recover_button_, replan_button_, execute_path_button_, discard_path_button_,
        clear_waypoints_button_, undo_waypoint_button_, reverse_waypoints_button_,
        clear_zones_button_, undo_zone_button_, save_map_button_,
        republish_maps_button_, regenerate_maps_button_, update_footprint_button_,
        calibrate_geometry_button_, mls_delete_button_, mls_fill_button_,
        regen_travmap_button_, auto_plan_check_,
        clear_executed_path_button_, record_bag_button_,
        plan_return_button_, return_mode_combo_,
        waypoint_index_spin_, delete_waypoint_button_, truncate_waypoints_button_,
        truncate_before_button_, plan_from_button_,
        zone_index_spin_, delete_zone_button_,
        fill_z_spin_, fill_roll_spin_, fill_pitch_spin_, fill_auto_check_, delete_top_spin_,
        groom_check_, groom_top_spin_, groom_margin_spin_,
        save_mission_button_, load_mission_button_, recalibrate_height_button_};

    // Configuration-style groups (rarely touched mid-mission) sit below all
    // mission controls, at the end of the scrollable content.
    layout->addWidget(checks_group_);
    layout->addWidget(controllers_group_);

    layout->addStretch();

    // The content outgrew typical screen heights, which also stopped RViz
    // from shrinking the panel (a panel's minimum size bounds the window).
    // Everything except the pinned Pause/Resume/Start stack lives in a scroll
    // area, so the panel can be resized down to roughly that stack alone and
    // the mission-flow controls never require scrolling (misclick hazard on
    // small screens). Abort mission (ex-STOP) lives in the grid: it is a
    // deliberate mission-ending action, not the quick-stop — that is Pause.
    auto* content = new QWidget;
    content->setLayout(layout);
    auto* scroll = new QScrollArea;
    scroll->setWidget(content);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    // Big, color-coded, pinned, stacked full-width: these are the two
    // mid-mission controls, so they must be hittable without scrolling or
    // aiming. Grey when disabled -- a colored-but-dead button invites the
    // misclicks this stack exists to avoid.
    pause_button_->setMinimumHeight(52);
    pause_button_->setStyleSheet(
        "QPushButton { font-weight: bold; font-size: 16px;"
        " background-color: #e65100; color: white; padding: 6px; }"
        "QPushButton:disabled { background-color: #4a4a4a; color: #9e9e9e; }");
    resume_button_->setMinimumHeight(52);
    resume_button_->setStyleSheet(
        "QPushButton { font-weight: bold; font-size: 16px;"
        " background-color: #2e7d32; color: white; padding: 6px; }"
        "QPushButton:disabled { background-color: #4a4a4a; color: #9e9e9e; }");
    // Start (send the reviewed path / resume the retained mission — see
    // onExecutePath): third member of the pinned stack, blue to read as
    // "go" without colliding with Resume's green.
    execute_path_button_->setMinimumHeight(52);
    execute_path_button_->setStyleSheet(
        "QPushButton { font-weight: bold; font-size: 16px;"
        " background-color: #1565c0; color: white; padding: 6px; }"
        "QPushButton:disabled { background-color: #4a4a4a; color: #9e9e9e; }");
    // Plan from WP: fourth member of the pinned stack, paired with the WP #
    // spin box it (and the waypoint Delete buttons in the scroll area) reads.
    plan_from_button_->setMinimumHeight(52);
    plan_from_button_->setStyleSheet(
        "QPushButton { font-weight: bold; font-size: 16px;"
        " background-color: #00695c; color: white; padding: 6px; }"
        "QPushButton:disabled { background-color: #4a4a4a; color: #9e9e9e; }");
    waypoint_index_spin_->setMinimumHeight(52);
    auto* plan_from_row = new QHBoxLayout;
    plan_from_row->setContentsMargins(0, 0, 0, 0);
    plan_from_row->setSpacing(2);
    plan_from_row->addWidget(waypoint_index_spin_);
    plan_from_row->addWidget(plan_from_button_, 1);
    auto* pause_resume_row = new QVBoxLayout;
    pause_resume_row->setContentsMargins(0, 0, 0, 0);
    pause_resume_row->setSpacing(2);
    pause_resume_row->addWidget(pause_button_);
    pause_resume_row->addWidget(resume_button_);
    pause_resume_row->addWidget(execute_path_button_);
    pause_resume_row->addLayout(plan_from_row);
    auto* outer = new QVBoxLayout;
    outer->setContentsMargins(0, 0, 0, 0);
    outer->addLayout(pause_resume_row);
    outer->addWidget(scroll);
    setLayout(outer);

    connect(stop_button_, &QPushButton::clicked, this, &OperatorPanel::onStopExecution);
    connect(pause_button_, &QPushButton::clicked, this, &OperatorPanel::onPauseExecution);
    connect(resume_button_, &QPushButton::clicked, this, &OperatorPanel::onResumeExecution);
    connect(recover_button_, &QPushButton::clicked, this, &OperatorPanel::onRecoverMission);
    connect(replan_button_, &QPushButton::clicked, this, &OperatorPanel::onReplanMission);
    connect(plan_from_button_, &QPushButton::clicked, this, &OperatorPanel::onPlanFromWaypoint);
    connect(execute_path_button_, &QPushButton::clicked, this, &OperatorPanel::onExecutePath);
    connect(discard_path_button_, &QPushButton::clicked, this, &OperatorPanel::onDiscardPath);
    connect(delete_waypoint_button_, &QPushButton::clicked, this, &OperatorPanel::onDeleteWaypoint);
    connect(truncate_waypoints_button_, &QPushButton::clicked, this, &OperatorPanel::onTruncateWaypoints);
    connect(truncate_before_button_, &QPushButton::clicked, this, &OperatorPanel::onTruncateWaypointsBefore);
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
    connect(recalibrate_height_button_, &QPushButton::clicked, this, [this]() {
        callTrigger(recalibrate_height_client_, "Recalibrate height");
    });
    connect(clear_executed_path_button_, &QPushButton::clicked, this, [this]() {
        callTrigger(clear_executed_path_client_, "Clear executed path");
    });
    connect(calibrate_geometry_button_, &QPushButton::clicked, this, [this]() {
        callTrigger(calibrate_geometry_client_, "Calibrate geometry");
    });
    connect(mls_delete_button_, &QPushButton::clicked, this, [this]() { onDeleteMlsPatches(); });
    connect(mls_fill_button_, &QPushButton::clicked, this, [this]() { onFillPlane(); });
    connect(regen_travmap_button_, &QPushButton::clicked, this, [this]() {
        callTrigger(regen_travmap_client_, "Regenerate trav map");
    });
    connect(groom_top_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) {
        if (!planner_param_client_ || !planner_param_client_->service_is_ready()){
            setStatusText("Groom ceiling: planner parameter service unavailable.");
            return;
        }
        const double v = groom_top_spin_->value();
        planner_param_client_->set_parameters(
            {rclcpp::Parameter("groom_delete_top", v)},
            [this, v, guard = QPointer<OperatorPanel>(this)](
                std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future)
            {
                if (!guard){
                    return;
                }
                const auto results = future.get();
                const bool ok = !results.empty() && results[0].successful;
                QMetaObject::invokeMethod(this, [this, v, ok]() {
                    setStatusText(ok
                        ? QString("Groom ceiling set to %1 m").arg(v, 0, 'f', 1)
                        : QString("Groom ceiling REJECTED by the planner (old build running?)"));
                }, Qt::QueuedConnection);
            });
    });
    connect(groom_margin_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) {
        if (!planner_param_client_ || !planner_param_client_->service_is_ready()){
            setStatusText("Groom margin: planner parameter service unavailable.");
            return;
        }
        const double v = groom_margin_spin_->value();
        planner_param_client_->set_parameters(
            {rclcpp::Parameter("groom_margin", v)},
            [this, v, guard = QPointer<OperatorPanel>(this)](
                std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future)
            {
                if (!guard){
                    return;
                }
                const auto results = future.get();
                const bool ok = !results.empty() && results[0].successful;
                QMetaObject::invokeMethod(this, [this, v, ok]() {
                    setStatusText(ok
                        ? QString("Groom margin set to %1 m").arg(v, 0, 'f', 2)
                        : QString("Groom margin REJECTED by the planner (old build running?)"));
                }, Qt::QueuedConnection);
            });
    });
    connect(groom_check_, &QCheckBox::toggled, this, [this](bool checked) {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = checked;
        if (set_groom_client_->service_is_ready()){
            set_groom_client_->async_send_request(request);
        } else {
            setStatusText("Grooming service unavailable (planner not running?)");
        }
    });
    connect(auto_plan_check_, &QCheckBox::toggled, this, [this](bool checked) {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = checked;
        if (set_auto_plan_client_->service_is_ready()){
            set_auto_plan_client_->async_send_request(request);
        } else {
            setStatusText("Auto-plan service unavailable (planner not running?)");
        }
    });
    connect(pause_at_wp_check_, &QCheckBox::toggled, this, [this](bool checked) {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = checked;
        if (set_pause_at_wp_client_->service_is_ready()){
            set_pause_at_wp_client_->async_send_request(request);
        } else {
            setStatusText("Pause-at-waypoints service unavailable (follower not running?)");
        }
    });
    connect(record_bag_button_, &QPushButton::clicked, this, [this](bool checked) {
        // The latched status topic is the truth; the check state follows it.
        callTrigger(checked ? start_bag_client_ : stop_bag_client_,
                    checked ? "Start bag recording" : "Stop bag recording");
    });
    connect(update_footprint_button_, &QPushButton::clicked, this, [this]() {
        callTrigger(update_footprint_client_, "Update footprint");
    });
    connect(save_mission_button_, &QPushButton::clicked, this, &OperatorPanel::onSaveMission);
    connect(load_mission_button_, &QPushButton::clicked, this, &OperatorPanel::onLoadMission);
    connect(refresh_controllers_button_, &QPushButton::clicked, this,
            &OperatorPanel::refreshControllers);
    connect(nav_controllers_button_, &QPushButton::clicked, this, [this]() {
        switchControllers(kNavigationControllers, {}, "Enable navigation controllers");
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
    // Latched planner truth "a fresh, not-yet-executed preview exists". Drives
    // the Execute button's meaning: with a preview Execute promotes it; without
    // one it acts as Resume (see onExecutePath).
    preview_pending_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/ugv_nav4d_ros2/preview_pending", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            QMetaObject::invokeMethod(this, [this, pending = msg->data]() {
                preview_pending_ = pending;
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
    deviation_text_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/follow_path_client/deviation_text", 10,
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            const QString text = "Deviation: " + QString::fromStdString(msg->data);
            // color from "value / limit": green in the OK band, orange when
            // past half the limit, red at breach
            float value = 0.0f;
            float limit = 0.0f;
            QString style;
            if (std::sscanf(msg->data.c_str(), "%f / %f", &value, &limit) == 2 &&
                limit > 0.0f)
            {
                const float ratio = value / limit;
                style = ratio >= 1.0f ? "QLabel { color: #e0564f; font-weight: bold; }"
                      : ratio >= 0.5f ? "QLabel { color: #e8a33c; font-weight: bold; }"
                                      : "QLabel { color: #4fc26b; }";
            }
            QMetaObject::invokeMethod(this, [this, text, style]() {
                deviation_label_->setText(text);
                deviation_label_->setStyleSheet(style);
            }, Qt::QueuedConnection);
        });
    footprint_info_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/ugv_nav4d_ros2/footprint_info", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            const QString text = "Footprint: " + QString::fromStdString(msg->data);
            QMetaObject::invokeMethod(this, [this, text]() {
                footprint_label_->setText(text);
                footprint_label_->setToolTip(text);
            }, Qt::QueuedConnection);
        });
    rebuilding_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/ugv_nav4d_ros2/rebuilding", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            QMetaObject::invokeMethod(this, [this, rebuilding = msg->data]() {
                rebuilding_ = rebuilding;
                for (auto* button : rebuild_sensitive_buttons_){
                    if (button){
                        button->setEnabled(!rebuilding);
                    }
                }
                if (rebuilding){
                    setStatusText("Rebuilding maps and planner... buttons disabled until done.");
                } else {
                    // restore state-dependent enables instead of blanket-on
                    updateExecuteEnabled();
                }
            }, Qt::QueuedConnection);
        });
    waypoint_poses_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
        "/ugv_nav4d_ros2/waypoint_poses", rclcpp::QoS(1).transient_local(),
        [this](const geometry_msgs::msg::PoseArray::SharedPtr msg)
        {
            const int count = static_cast<int>(msg->poses.size());
            QMetaObject::invokeMethod(this, [this, count]() {
                // Only meaningful for waypoint routes: no queue, no toggle.
                pause_at_wp_check_->setEnabled(count > 0);
                if (count == 0){
                    pause_at_wp_check_->setToolTip(
                        "Add at least one waypoint first; a plain single-goal "
                        "route has nothing to pause at.");
                }
                // Deliberately NOT clamping the spin box maximum to the queue
                // size: the count can lag over the bridge and fighting the
                // operator's typing is worse than the planner's own clear
                // out-of-range reply ("Waypoint 200 does not exist (20
                // queued)."). The count only gates whether the waypoint
                // actions are meaningful at all.
                waypoint_count_ = count;
                updateExecuteEnabled();
            }, Qt::QueuedConnection);
        });
    pause_at_wp_state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/follow_path_client/pause_at_waypoints", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            QMetaObject::invokeMethod(this, [this, on = msg->data]() {
                // Follow the follower's latched truth without re-triggering
                // the service call.
                QSignalBlocker block(pause_at_wp_check_);
                pause_at_wp_check_->setChecked(on);
            }, Qt::QueuedConnection);
        });
    auto_plan_state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/ugv_nav4d_ros2/auto_plan", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            QMetaObject::invokeMethod(this, [this, on = msg->data]() {
                // Follow the planner's latched truth without re-triggering
                // the service call.
                QSignalBlocker block(auto_plan_check_);
                auto_plan_check_->setChecked(on);
            }, Qt::QueuedConnection);
        });
    groom_state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/ugv_nav4d_ros2/groom_under_robot", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            QMetaObject::invokeMethod(this, [this, on = msg->data]() {
                QSignalBlocker block(groom_check_);
                groom_check_->setChecked(on);
            }, Qt::QueuedConnection);
        });
    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/arter/mcs/cmd_vel", rclcpp::QoS(10),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            const QString text = QString("cmd_vel: %1 m/s | %2 rad/s")
                .arg(msg->linear.x, 0, 'f', 2)
                .arg(msg->angular.z, 0, 'f', 2);
            QMetaObject::invokeMethod(this, [this, text]() {
                cmd_vel_label_->setText(text);
            }, Qt::QueuedConnection);
        });
    bag_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/ugv_nav4d_ros2/bag_recorder_status", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            const bool recording = msg->data.rfind("RECORDING", 0) == 0;
            const QString text = "Bag: " + QString::fromStdString(msg->data);
            QMetaObject::invokeMethod(this, [this, text, recording]() {
                bag_label_->setText(text);
                bag_label_->setStyleSheet(recording
                    ? "QLabel { color: #e0564f; font-weight: bold; }" : "");
                record_bag_button_->setChecked(recording);
                record_bag_button_->setText(recording
                    ? "\u25A0 Stop nav bag" : "\u25CF Record nav bag");
                record_bag_button_->setStyleSheet(recording
                    ? "QPushButton { font-weight: bold; background-color: #b71c1c; color: white; }"
                    : "");
            }, Qt::QueuedConnection);
        });
    height_info_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/ugv_nav4d_ros2/height_info", 10,
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            const QString text = "Height: " + QString::fromStdString(msg->data);
            QString style;
            if (msg->data.find("INVALID") != std::string::npos){
                style = "QLabel { color: #e0564f; font-weight: bold; }";
            } else if (msg->data.find("check height") != std::string::npos){
                style = "QLabel { color: #e8a33c; font-weight: bold; }";
            } else if (msg->data.find("(ok)") != std::string::npos){
                style = "QLabel { color: #4fc26b; }";
            }
            QMetaObject::invokeMethod(this, [this, text, style]() {
                height_label_->setText(text);
                height_label_->setStyleSheet(style);
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
    save_map_client_ = node_->create_client<ugv_nav4d_ros2::srv::MissionFile>("/ugv_nav4d_ros2/save_mls_map");
    map_publish_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/map_publish");
    regenerate_maps_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/regenerate_maps");
    update_footprint_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/update_footprint");
    recalibrate_height_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/recalibrate_height");
    set_pause_at_wp_client_ = node_->create_client<std_srvs::srv::SetBool>("/ugv_nav4d_ros2/set_pause_at_waypoints");
    set_auto_plan_client_ = node_->create_client<std_srvs::srv::SetBool>("/ugv_nav4d_ros2/set_auto_plan");
    set_groom_client_ = node_->create_client<std_srvs::srv::SetBool>("/ugv_nav4d_ros2/set_groom_under_robot");
    planner_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/ugv_nav4d_ros2");
    // Seed the spinbox with the planner's current value once it is reachable.
    planner_param_client_->wait_for_service(std::chrono::seconds(0));
    planner_param_client_->get_parameters({"groom_delete_top", "groom_margin"},
        [this, guard = QPointer<OperatorPanel>(this)](
            std::shared_future<std::vector<rclcpp::Parameter>> future)
        {
            if (!guard){
                return;
            }
            const auto params = future.get();
            if (params.size() >= 2){
                QMetaObject::invokeMethod(this,
                    [this, top = params[0].as_double(), margin = params[1].as_double()]() {
                        QSignalBlocker block_top(groom_top_spin_);
                        groom_top_spin_->setValue(top);
                        QSignalBlocker block_margin(groom_margin_spin_);
                        groom_margin_spin_->setValue(margin);
                    }, Qt::QueuedConnection);
            }
        });
    clear_executed_path_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/clear_executed_path");
    calibrate_geometry_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/calibrate_geometry");
    mls_delete_client_ = node_->create_client<ugv_nav4d_ros2::srv::DeleteMlsPatches>("/ugv_nav4d_ros2/mls_delete_last_zone");
    mls_fill_client_ = node_->create_client<ugv_nav4d_ros2::srv::SetFillPlane>("/ugv_nav4d_ros2/mls_fill_last_zone");
    regen_travmap_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/regenerate_travmap");
    start_bag_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/start_bag_recording");
    stop_bag_client_ = node_->create_client<std_srvs::srv::Trigger>("/ugv_nav4d_ros2/stop_bag_recording");
    save_mission_client_ = node_->create_client<ugv_nav4d_ros2::srv::MissionFile>("/ugv_nav4d_ros2/save_mission");
    load_mission_client_ = node_->create_client<ugv_nav4d_ros2::srv::MissionFile>("/ugv_nav4d_ros2/load_mission");

    list_controllers_client_ = node_->create_client<controller_manager_msgs::srv::ListControllers>(
        "/arter/ros_control/controller_manager/list_controllers");
    switch_controller_client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
        "/arter/ros_control/controller_manager/switch_controller");
    // Controllers are queried ON DEMAND only (Refresh button, and after a
    // switch request): a periodic ListControllers poll would cost a service
    // round-trip over the WiFi hop every few seconds for information the
    // operator rarely watches.
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
        [this, guard = QPointer<OperatorPanel>(this)](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future)
        {
            // The panel can be destroyed (rviz closed) while a request to a slow
            // service is in flight; the response must not touch a dangling this.
            if (!guard)
            {
                return;
            }
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
        [this, actionName, guard = QPointer<OperatorPanel>(this)](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future)
        {
            // The panel can be destroyed (rviz closed) while a request to a slow
            // service is in flight; the response must not touch a dangling this.
            if (!guard)
            {
                return;
            }
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

    // Start needs a FRESH preview (replan/recovery output confirmed by the
    // operator). While paused without one, Resume is the single lit "go"
    // button — Start greys out instead of duplicating it (two enabled
    // identical buttons read as a choice that does not exist). The
    // Start-as-Resume dispatch in onExecutePath stays as a safety net for a
    // stale preview_pending over the bridge.
    QString blocking_check;
    const bool health_ok = health_received_ && checkedReadinessOk(&blocking_check);
    const bool resumable = paused && execution_can_resume_;
    const bool execute_ok = health_ok && !executing && !rebuilding_ &&
                            preview_pending_ && route_ready_;
    execute_path_button_->setEnabled(execute_ok);
    execute_path_button_->setToolTip(
        executing
            ? "A mission is already executing; pause or stop it first."
            : execute_ok
                ? (resumable
                    ? "Send the NEW path to the controller (Resume would "
                      "continue the old mission instead)."
                    : "Send the reviewed path to the controller.")
                : !health_received_
                    ? "Waiting for the first system-health report."
                    : !blocking_check.isEmpty()
                        ? QString("Blocked by readiness check \"%1\" — fix it or "
                                  "uncheck it under Execute gating checks.").arg(blocking_check)
                        : resumable
                            ? "No new path planned — use Resume."
                            : "Execution requires a route preview (Replan first).");

    // Waypoint-number actions need an actual queue; greyed out otherwise
    // (also re-applied here after the rebuild lock-out blanket-enables).
    const bool has_waypoints = waypoint_count_ > 0 && !rebuilding_;
    waypoint_index_spin_->setEnabled(has_waypoints);
    plan_from_button_->setEnabled(has_waypoints);
    delete_waypoint_button_->setEnabled(has_waypoints);
    truncate_waypoints_button_->setEnabled(has_waypoints);
    truncate_before_button_->setEnabled(has_waypoints);

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

void OperatorPanel::onDeleteMlsPatches()
{
    if (!mls_delete_client_ || !node_)
    {
        return;
    }
    if (!mls_delete_client_->service_is_ready())
    {
        setStatusText("Delete MLS patches: service unavailable (is the planner node running?)");
        return;
    }
    setStatusText("Delete MLS patches requested...");
    auto request = std::make_shared<ugv_nav4d_ros2::srv::DeleteMlsPatches::Request>();
    request->top_m = delete_top_spin_->value();
    mls_delete_client_->async_send_request(request,
        [this, guard = QPointer<OperatorPanel>(this)](rclcpp::Client<ugv_nav4d_ros2::srv::DeleteMlsPatches>::SharedFuture future)
        {
            if (!guard)
            {
                return;
            }
            const auto response = future.get();
            setStatusText(QString::fromStdString(response->message));
        });
}

void OperatorPanel::onFillPlane()
{
    if (!mls_fill_client_ || !node_)
    {
        return;
    }
    if (!mls_fill_client_->service_is_ready())
    {
        setStatusText("Fill plane: service unavailable (is the planner node running?)");
        return;
    }
    setStatusText("Fill plane requested...");
    auto request = std::make_shared<ugv_nav4d_ros2::srv::SetFillPlane::Request>();
    request->auto_fit = fill_auto_check_ && fill_auto_check_->isChecked();
    request->z_offset = fill_z_spin_->value();
    request->roll_deg = fill_roll_spin_->value();
    request->pitch_deg = fill_pitch_spin_->value();
    request->delete_top_m = delete_top_spin_->value();
    mls_fill_client_->async_send_request(request,
        [this, guard = QPointer<OperatorPanel>(this)](rclcpp::Client<ugv_nav4d_ros2::srv::SetFillPlane>::SharedFuture future)
        {
            if (!guard)
            {
                return;
            }
            const auto response = future.get();
            setStatusText(QString::fromStdString(response->message));
        });
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
        [this, actionName, guard = QPointer<OperatorPanel>(this)](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
        {
            // The panel can be destroyed (rviz closed) while a request to a slow
            // service is in flight; the response must not touch a dangling this.
            if (!guard)
            {
                return;
            }
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
    // Same ordering rationale as Execute: a pause can outlive a controller
    // deactivation (e-stop cycle, manual switch), so Resume re-arms the set.
    callTrigger(resume_execution_client_, "Resume");
    activateNavigationControllers("resume requested");
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
        [this, guard = QPointer<OperatorPanel>(this)](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
        {
            // The panel can be destroyed (rviz closed) while a request to a slow
            // service is in flight; the response must not touch a dangling this.
            if (!guard)
            {
                return;
            }
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
        [this, guard = QPointer<OperatorPanel>(this)](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
        {
            // The panel can be destroyed (rviz closed) while a request to a slow
            // service is in flight; the response must not touch a dangling this.
            if (!guard)
            {
                return;
            }
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

void OperatorPanel::activateNavigationControllers(const QString& context)
{
    //Command FIRST, controller switch right after (operator-chosen order): the
    //follower/goal handshake takes longer than the switch, so the controllers
    //are up before the first command lands, and the button is never delayed
    //by the switch round-trip. BEST_EFFORT makes the activation a no-op when
    //they already run; without a controller_manager (bench/sim) the switch is
    //skipped entirely.
    if (!switch_controller_client_ || !switch_controller_client_->service_is_ready())
    {
        return;
    }
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    for (const auto& name : kNavigationControllers)
    {
        request->activate_controllers.push_back(name.toStdString());
    }
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    switch_controller_client_->async_send_request(request,
        [this, guard = QPointer<OperatorPanel>(this), context](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future)
        {
            if (!guard)
            {
                return;
            }
            const bool ok = future.get()->ok;
            QMetaObject::invokeMethod(this, [this, ok, context]() {
                refreshControllers();
                if (!ok)
                {
                    setStatusText("WARNING: " + context +
                                  ", but the navigation controllers could not be enabled.");
                }
            }, Qt::QueuedConnection);
        });
}

void OperatorPanel::onExecutePath()
{
    // Without a fresh preview, re-sending the stored path would restart the
    // mission from its ORIGINAL start segment — far behind a partially-driven
    // robot, where the controller cannot lock on and aborts ("Execute does
    // nothing" field symptom). The retained mission is what the operator
    // wants continued, so act as Resume instead.
    if (!preview_pending_ &&
        execution_state_ == ugv_nav4d_ros2::msg::MissionStatus::PAUSED &&
        execution_can_resume_)
    {
        setStatusText("No new preview pending — resuming the retained mission "
                      "from the robot position.");
        onResumeExecution();
        return;
    }
    callTrigger(execute_path_client_, "Execute path");
    activateNavigationControllers("path executing");
}

void OperatorPanel::onDiscardPath()
{
    callTrigger(discard_path_client_, "Discard path");
}

void OperatorPanel::sendWaypointEdit(bool truncate_after, bool truncate_before, const QString& action)
{
    if (!edit_waypoint_client_ || !node_)
    {
        return;
    }
    if (!edit_waypoint_client_->service_is_ready())
    {
        setStatusText(action + ": service unavailable (is the planner node running?)");
        return;
    }
    auto request = std::make_shared<ugv_nav4d_ros2::srv::EditWaypoint::Request>();
    request->index = static_cast<uint32_t>(waypoint_index_spin_->value());
    request->remove = !truncate_after && !truncate_before;
    request->truncate_after = truncate_after;
    request->truncate_before = truncate_before;
    edit_waypoint_client_->async_send_request(request,
        [this, guard = QPointer<OperatorPanel>(this)](rclcpp::Client<ugv_nav4d_ros2::srv::EditWaypoint>::SharedFuture future)
        {
            // The panel can be destroyed (rviz closed) while a request to a slow
            // service is in flight; the response must not touch a dangling this.
            if (!guard)
            {
                return;
            }
            setStatusText(QString::fromStdString(future.get()->message));
        });
}

void OperatorPanel::onDeleteWaypoint()
{
    sendWaypointEdit(false, false, "Delete waypoint");
}

void OperatorPanel::onPlanFromWaypoint()
{
    if (!edit_waypoint_client_ || !node_)
    {
        return;
    }
    if (!edit_waypoint_client_->service_is_ready())
    {
        setStatusText("Plan from WP #: service unavailable (is the planner node running?)");
        return;
    }
    auto request = std::make_shared<ugv_nav4d_ros2::srv::EditWaypoint::Request>();
    request->index = static_cast<uint32_t>(waypoint_index_spin_->value());
    request->truncate_before = true;
    edit_waypoint_client_->async_send_request(request,
        [this, guard = QPointer<OperatorPanel>(this)](rclcpp::Client<ugv_nav4d_ros2::srv::EditWaypoint>::SharedFuture future)
        {
            if (!guard)
            {
                return;
            }
            const auto result = future.get();
            if (!result->success)
            {
                setStatusText(QString::fromStdString(result->message));
                return;
            }
            // Queue now starts at the entered waypoint. With auto-plan on, the
            // queue change itself already replanned the preview; otherwise
            // trigger the replan-from-robot explicitly.
            QMetaObject::invokeMethod(this, [this]() {
                if (auto_plan_check_ && auto_plan_check_->isChecked())
                {
                    setStatusText("Plan from WP #: waypoints dropped; auto-plan "
                                  "is producing the preview.");
                    return;
                }
                callTrigger(replan_mission_client_, "Plan from WP #");
            }, Qt::QueuedConnection);
        });
}

void OperatorPanel::onTruncateWaypoints()
{
    sendWaypointEdit(true, false, "Delete after");
}

void OperatorPanel::onTruncateWaypointsBefore()
{
    sendWaypointEdit(false, true, "Delete before");
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
        [this, guard = QPointer<OperatorPanel>(this)](rclcpp::Client<ugv_nav4d_ros2::srv::DeleteForbiddenZone>::SharedFuture future)
        {
            // The panel can be destroyed (rviz closed) while a request to a slow
            // service is in flight; the response must not touch a dangling this.
            if (!guard)
            {
                return;
            }
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
        [this, guard = QPointer<OperatorPanel>(this)](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
        {
            // The panel can be destroyed (rviz closed) while a request to a slow
            // service is in flight; the response must not touch a dangling this.
            if (!guard)
            {
                return;
            }
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
    //Mirrors Save mission: pick the FULL destination path; the planner node
    //writes the file. Path is resolved on the node's machine.
    QString file = QFileDialog::getSaveFileName(
        this, "Save MLS map",
        last_mission_dir_.isEmpty() ? QString("mls_map.bin")
                                    : last_mission_dir_ + "/mls_map.bin",
        "MLS map files (*.bin);;All files (*)");
    if (file.isEmpty())
    {
        return; // dialog cancelled
    }
    last_mission_dir_ = QFileInfo(file).absolutePath();
    callMissionFile(save_map_client_, file, "Save map");
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
        [this, actionName, guard = QPointer<OperatorPanel>(this)](rclcpp::Client<ugv_nav4d_ros2::srv::MissionFile>::SharedFuture future)
        {
            // The panel can be destroyed (rviz closed) while a request to a slow
            // service is in flight; the response must not touch a dangling this.
            if (!guard)
            {
                return;
            }
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
