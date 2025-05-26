#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose.hpp> //  +++
#include <functional>
#include <thread>
#include <atomic>
#include <chrono>

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

        declare_parameter<std::vector<double>>("mock_cubes_xy", {});
        declare_parameter<double>("z_offset", 0.10);
    }
    ~CubePointerController() override
    {
        stop_thread_.store(true);
        if (sequence_thread_.joinable())
            sequence_thread_.join();
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

        srv_stop_ = create_service<Trigger>(
            "stop",
            [this](
                const std::shared_ptr<Trigger::Request> req, //  ✱ explicit
                std::shared_ptr<Trigger::Response> res)      //  ✱ explicit
            {
                (void)req;           // we don’t use the request body
                move_group_->stop(); // cancel active trajectory
                paused_.store(true);
                res->success = true;
                res->message = "Motion stopped; sequence paused.";
                RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
            });

        srv_resume_ = create_service<Trigger>(
            "resume",
            [this](
                const std::shared_ptr<Trigger::Request> req, //  ✱ explicit
                std::shared_ptr<Trigger::Response> res)      //  ✱ explicit
            {
                (void)req;
                if (!paused_)
                {
                    res->success = false;
                    res->message = "Sequence is not paused.";
                    return;
                }
                paused_.store(false);
                res->success = true;
                res->message = "Resumed.";
                RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
            });

        srv_restart_ = create_service<Trigger>(
            "restart",
            [this](const std::shared_ptr<Trigger::Request> req,
                   std::shared_ptr<Trigger::Response> res)
            {
                (void)req;
                paused_.store(true);          // freeze worker
                restart_requested_.store(true); // flag for thread
                res->success = true;
                res->message = "Sequence restart requested.";
                RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
            });

        RCLCPP_INFO(get_logger(), "Services [/stop] [/resume] [/restart] ready.");

        loadMockCubes(); // +++
        sequence_thread_ = std::thread([this]
                                       {
        // run until node is shutting down
        while (!stop_thread_.load()) {
            if (restart_requested_.load()) {
                // reload cubes & state
                cube_poses_.clear();
                loadMockCubes();          // (later: pull fresh /cube_tf list)
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
    // ---------- service callback ----------------------------------
    void handleGoHome(const std::shared_ptr<Trigger::Request>,
                      std::shared_ptr<Trigger::Response> res)
    {
        move_group_->stop();
        paused_.store(true);

        move_group_->clearPoseTargets();
        move_group_->setStartStateToCurrentState();

        std::size_t dof = move_group_->getJointNames().size();
        if (home_joints_.size() != dof)
        {
            res->success = false;
            res->message = "home_joint_values length mismatch";
            RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
            return;
        }

        move_group_->setJointValueTarget(home_joints_);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            res->success = false;
            res->message = "Planning failed";
            return;
        }
        auto exec = move_group_->execute(plan);
        res->success = (exec == moveit::core::MoveItErrorCode::SUCCESS);
        res->message = res->success ? "Reached home" : "Execution aborted";
    }

    void loadMockCubes()
    {
        auto flat = get_parameter("mock_cubes_xy").as_double_array();
        double z = get_parameter("z_offset").as_double();

        for (size_t i = 0; i + 1 < flat.size(); i += 2)
        {
            geometry_msgs::msg::Pose p;
            p.position.x = flat[i];
            p.position.y = flat[i + 1];
            p.position.z = z;

            // tool pointing straight down: 180° about X  ->  quaternion (1,0,0,0)
            p.orientation.w = 0.0;
            p.orientation.x = 1.0;
            p.orientation.y = 0.0;
            p.orientation.z = 0.0;

            cube_poses_.push_back(p);
        }
    }
    void planNextCube()
    {
        if (paused_.load())
            return; // wait for /resume

        if (current_index_ >= cube_poses_.size())
        {
            // Finished all cubes – return home
            paused_.store(false);
            handleGoHome(nullptr, std::make_shared<Trigger::Response>());
            return;
        }

        move_group_->stop();
        move_group_->clearPoseTargets();
        move_group_->setStartStateToCurrentState();
        move_group_->setPoseTarget(cube_poses_[current_index_]);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            current_index_++;
            auto exec_res = move_group_->execute(plan); // blocking

            if (exec_res != moveit::core::MoveItErrorCode::SUCCESS)
                RCLCPP_WARN(get_logger(), "Execution to cube %zu aborted.", current_index_ - 1);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Planning to cube %zu failed, skipping.", current_index_);
            current_index_++;
        }
    }

    // ---------- members -------------------------------------------
    std::vector<double> home_joints_;

    std::vector<geometry_msgs::msg::Pose> cube_poses_;
    size_t current_index_ = 0;
    std::thread sequence_thread_;

    std::atomic_bool paused_{false};
    std::atomic_bool stop_thread_{false}; // for clean shutdown

    rclcpp::Service<Trigger>::SharedPtr srv_stop_;
    rclcpp::Service<Trigger>::SharedPtr srv_resume_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Service<Trigger>::SharedPtr srv_go_home_;

    rclcpp::Service<Trigger>::SharedPtr srv_restart_;
    std::atomic_bool restart_requested_{false};
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
