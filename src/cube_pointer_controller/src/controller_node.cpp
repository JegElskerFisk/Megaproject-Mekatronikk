#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

using Trigger = std_srvs::srv::Trigger;

class CubePointerController : public rclcpp::Node
{
public:
  CubePointerController()
  : Node("cube_pointer_controller")
  {
    // Declare parameters; we’ll read them again in initMoveGroup()
    declare_parameter<std::vector<double>>("home_joint_values",
                                           std::vector<double>{});
    declare_parameter<double>("velocity_scaling", 0.2);
    declare_parameter<double>("acceleration_scaling", 0.2);
  }

  /** must be called once AFTER the node is wrapped in a shared_ptr */
  void initMoveGroup()
  {
    // ── parameters ───────────────────────────────────────────────
    home_joints_ = get_parameter("home_joint_values")
                     .as_double_array();

    double vel_scaling = get_parameter("velocity_scaling").as_double();
    double acc_scaling = get_parameter("acceleration_scaling").as_double();

    // ── Move-It handle ───────────────────────────────────────────
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "ur_manipulator");

    move_group_->setMaxVelocityScalingFactor(vel_scaling);
    move_group_->setMaxAccelerationScalingFactor(acc_scaling);
    RCLCPP_INFO(get_logger(), "MoveGroup ready.");

    // ── /go_home service ─────────────────────────────────────────
    srv_go_home_ = create_service<Trigger>(
        "go_home",
        std::bind(&CubePointerController::handleGoHome, this,
                  std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Service [/go_home] available.");
  }

private:
  // ---------- service callback ----------------------------------
  void handleGoHome(const std::shared_ptr<Trigger::Request>,
                    std::shared_ptr<Trigger::Response> res)
  {
    std::size_t dof = move_group_->getJointNames().size();
    if (home_joints_.size() != dof) {
      res->success = false;
      res->message = "home_joint_values length mismatch";
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
      return;
    }

    move_group_->setJointValueTarget(home_joints_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      res->success = false;
      res->message = "Planning failed";
      return;
    }
    auto exec = move_group_->execute(plan);
    res->success = (exec == moveit::core::MoveItErrorCode::SUCCESS);
    res->message = res->success ? "Reached home" : "Execution aborted";
  }

  // ---------- members -------------------------------------------
  std::vector<double> home_joints_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Service<Trigger>::SharedPtr srv_go_home_;
};

// ----------------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CubePointerController>();
  node->initMoveGroup();                 // ← crucial
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
