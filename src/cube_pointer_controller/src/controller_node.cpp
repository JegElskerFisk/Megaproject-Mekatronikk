#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <functional>
#include <thread>
#include <atomic>
#include <chrono>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <moveit_msgs/action/move_group.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using Trigger = std_srvs::srv::Trigger;
using MoveGroup = moveit_msgs::action::MoveGroup;
using GoalHandleMG = rclcpp_action::ClientGoalHandle<MoveGroup>;

class CubePointerController : public rclcpp::Node
{
public:
    CubePointerController()
        : Node("cube_pointer_controller")
    {

        declare_parameter<std::vector<double>>("home_joint_values",
                                               std::vector<double>{});

        declare_parameter<double>("z_offset", 0.20);
    }
    ~CubePointerController() override
    {
        stop_thread_.store(true);
        if (sequence_thread_.joinable())
            sequence_thread_.join();
    }

    void initMoveGroup()
    {
        // ── parameters
        home_joints_ = get_parameter("home_joint_values")
                           .as_double_array();

        joint_names_ = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"};

        // ── tf2 buffer & listener
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

        // ── move action server
        move_group_ac_ = rclcpp_action::create_client<MoveGroup>(
            shared_from_this(),
            "move_action");

        if (!move_group_ac_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(get_logger(),
                         "Could not contact move_group action server (did you launch ur_moveit_config?).");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(get_logger(), "Connected to external move_group action server");

        srv_restart_ = create_service<Trigger>(
            "restart",
            [this](const std::shared_ptr<Trigger::Request> req,
                   std::shared_ptr<Trigger::Response> res)
            {
                (void)req;
                paused_.store(true); // freeze worker
                restart_requested_.store(true);
                res->success = true;
                res->message = "Sequence restart requested.";
                RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
            });

        refreshCubeList();
        sequence_thread_ = std::thread([this]
                                       {
        // run until node is shutting down
        while (!stop_thread_.load()) {
            if (restart_requested_.load()) {
              
                cube_poses_.clear();
                refreshCubeList();          
                current_index_ = 0;
                paused_.store(false);
                restart_requested_.store(false);
                RCLCPP_INFO(get_logger(), "Sequence restart: starting from cube 0");
            }
            if (!paused_) {
                planNextCube();          // blocking execute() is fine here
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        } });
    }

private:
    bool sendPoseGoal(const geometry_msgs::msg::Pose &target_pose, size_t cube_index)
    {

        MoveGroup::Goal goal_msg;
        // Planning parameters
        goal_msg.request.group_name = "ur_manipulator";
        goal_msg.request.num_planning_attempts = 10;
        goal_msg.request.allowed_planning_time = 8.0;
        goal_msg.request.max_velocity_scaling_factor = 0.1;
        goal_msg.request.max_acceleration_scaling_factor = 0.1;
        goal_msg.request.pipeline_id = "pilz_industrial_motion_planner";
        goal_msg.request.planner_id = "PTP";

        moveit_msgs::msg::Constraints c;
        // position constraint:
        moveit_msgs::msg::PositionConstraint pc;
        pc.header.frame_id = "base_link";
        pc.link_name = "wrist_3_link"; // end‐effector link
        pc.target_point_offset.x = 0.0;
        pc.target_point_offset.y = 0.0;
        pc.target_point_offset.z = 0.0;
        pc.constraint_region.primitives.resize(1);
        pc.constraint_region.primitives[0].type =
            shape_msgs::msg::SolidPrimitive::SPHERE;
        pc.constraint_region.primitives[0].dimensions = {0.01};
        pc.constraint_region.primitive_poses.push_back(target_pose);
        pc.weight = 1.0;
        c.position_constraints.push_back(pc);

        // orientation constraint:
        moveit_msgs::msg::OrientationConstraint oc;
        oc.header.frame_id = "base_link";
        oc.link_name = "wrist_3_link";
        oc.orientation = target_pose.orientation;
        oc.absolute_x_axis_tolerance = 0.1;
        oc.absolute_y_axis_tolerance = 0.1;
        oc.absolute_z_axis_tolerance = 0.1;
        oc.weight = 1.0;
        c.orientation_constraints.push_back(oc);

        goal_msg.request.goal_constraints.push_back(c);

        auto send_opts = rclcpp_action::Client<MoveGroup>::SendGoalOptions{};
        send_opts.result_callback =
            [this, cube_index](const GoalHandleMG::WrappedResult &wrapped) {

            };

        auto goal_handle = move_group_ac_->async_send_goal(goal_msg, send_opts)
                               .get();
        if (!goal_handle)
            return false;

        auto wrapped = move_group_ac_->async_get_result(goal_handle)
                           .get();

        int code = static_cast<int>(wrapped.code);
        if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(get_logger(), "Cube %zu reached", cube_index);
            return true;
        }
        else
        {
            RCLCPP_WARN(get_logger(),
                        "Cube %zu failed: action %d, MoveIt error %d (%s)",
                        cube_index, code,
                        wrapped.result->error_code.val,
                        wrapped.result->error_code.message.c_str());
            return false;
        }
    }

    void sendJointGoal(
        const std::vector<std::string> &joint_names,
        const std::vector<double> &joint_positions,
        std::shared_ptr<Trigger::Response> res)
    {

        MoveGroup::Goal goal;
        goal.request.group_name = "ur_manipulator";
        goal.request.num_planning_attempts = 1;
        goal.request.allowed_planning_time = 2.0;

        moveit_msgs::msg::Constraints c;
        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            moveit_msgs::msg::JointConstraint jc;
            jc.joint_name = joint_names[i];
            jc.position = joint_positions[i];
            jc.tolerance_above = 0.001;
            jc.tolerance_below = 0.001;
            jc.weight = 1.0;
            c.joint_constraints.push_back(jc);
        }
        goal.request.goal_constraints.push_back(c);

        auto send_opts = rclcpp_action::Client<MoveGroup>::SendGoalOptions{};
        send_opts.result_callback =
            [this, res](const GoalHandleMG::WrappedResult &wrapped)
        {
            if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                res->success = true;
                res->message = "Reached home via MoveIt action";
            }
            else
            {
                res->success = false;
                res->message = "Home action failed";
            }
            RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
        };

        auto future = move_group_ac_->async_send_goal(std::move(goal), send_opts);

        if (!future.get())
        {
            res->success = false;
            res->message = "Home goal was rejected";
        }
    }

    void handleGoHome(
        const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> res)
    {
        paused_.store(true); // freeze worker
        current_index_ = 0;  // reset sequence index

        if (home_joints_.size() != joint_names_.size())
        {
            res->success = false;
            res->message = "home_joint_values length mismatch";
            RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
            rclcpp::shutdown();
            return;
        }

        sendJointGoal(joint_names_, home_joints_, res);
    }

    void refreshCubeList()
    {
        cube_poses_.clear();
        static const std::vector<std::string> names = {"cube1", "cube2", "cube3"};
        double z = get_parameter("z_offset").as_double();

        for (const auto &name : names)
        {
            geometry_msgs::msg::TransformStamped tf;
            try
            {
                tf = tf_buffer_.lookupTransform("base_link", name, rclcpp::Time(0));
            }
            catch (const tf2::TransformException &e)
            {
                continue;
            }
            geometry_msgs::msg::Pose p;
            p.position.x = tf.transform.translation.x;
            p.position.y = tf.transform.translation.y;
            p.position.z = z;
            p.orientation.x = 1.0;
            p.orientation.w = 0.0;
            p.orientation.y = 0.0;
            p.orientation.z = 0.0;

            cube_poses_.push_back(p);
        }
    }

    void planNextCube()
    {
        if (paused_.load())
            return;

        if (current_index_ >= cube_poses_.size())
        {
            handleGoHome(nullptr, std::make_shared<Trigger::Response>());
            return;
        }
        refreshCubeList();

        auto pose = cube_poses_[current_index_];
        size_t idx = current_index_;
        current_index_++;

        sendPoseGoal(pose, idx);
    }

    // ---------- members -------------------------------------------
    std::vector<double> home_joints_;
    std::vector<std::string> joint_names_;

    rclcpp_action::Client<MoveGroup>::SharedPtr move_group_ac_;

    std::vector<geometry_msgs::msg::Pose> cube_poses_;
    size_t current_index_ = 0;
    std::thread sequence_thread_;

    std::atomic_bool paused_{false};
    std::atomic_bool stop_thread_{false};

    rclcpp::Service<Trigger>::SharedPtr srv_stop_;
    rclcpp::Service<Trigger>::SharedPtr srv_resume_;

    rclcpp::Service<Trigger>::SharedPtr srv_go_home_;

    rclcpp::Service<Trigger>::SharedPtr srv_restart_;
    std::atomic_bool restart_requested_{false};

    tf2_ros::Buffer tf_buffer_{get_clock()};
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    GoalHandleMG::SharedPtr current_goal_handle_{nullptr};
};

// ----------------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CubePointerController>();
    node->initMoveGroup();

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}