#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

class ArmExecutor : public rclcpp::Node
{
public:
  ArmExecutor() : Node("arm_executor")
  {
    declare_parameter<std::string>("planning_group", "ur_manipulator");
    declare_parameter<std::string>("pose_reference_frame", "world");
    declare_parameter<double>("velocity_scaling", 0.2);
    declare_parameter<double>("acceleration_scaling", 0.2);
    declare_parameter<double>("planning_time_sec", 5.0);

    planning_group_ = get_parameter("planning_group").as_string();
    pose_reference_frame_ = get_parameter("pose_reference_frame").as_string();
    vel_scaling_ = get_parameter("velocity_scaling").as_double();
    acc_scaling_ = get_parameter("acceleration_scaling").as_double();
    planning_time_sec_ = get_parameter("planning_time_sec").as_double();

    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/vla_goal_pose", 10,
        std::bind(&ArmExecutor::on_goal, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "ArmExecutor ready. planning_group='%s', pose_reference_frame='%s'",
                planning_group_.c_str(), pose_reference_frame_.c_str());
  }

  void start()
  {
    // Safe to call shared_from_this() here (after make_shared in main)
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_);

    RCLCPP_INFO(get_logger(), "MoveIt planning frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(get_logger(), "MoveIt end effector link: %s", move_group_->getEndEffectorLink().c_str());

    move_group_->setPoseReferenceFrame(pose_reference_frame_); // official API :contentReference[oaicite:5]{index=5}
    move_group_->setMaxVelocityScalingFactor(vel_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acc_scaling_);
    move_group_->setPlanningTime(planning_time_sec_);

    worker_ = std::thread([this]() { this->worker_loop(); });
  }

  ~ArmExecutor() override
  {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      stop_ = true;
      cv_.notify_all();
    }
    if (worker_.joinable())
      worker_.join();
  }

private:
  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    latest_goal_ = *msg;
    cv_.notify_all();
  }

  void worker_loop()
  {
    // Give MoveIt/Gazebo a moment to finish starting up in some setups.
    rclcpp::sleep_for(std::chrono::seconds(2));

    while (rclcpp::ok())
    {
      geometry_msgs::msg::PoseStamped goal;
      {
        std::unique_lock<std::mutex> lk(mtx_);
        cv_.wait(lk, [&]() { return stop_ || latest_goal_.has_value(); });
        if (stop_)
          return;
        goal = *latest_goal_;
        latest_goal_.reset();
      }

      if (!move_group_)
      {
        RCLCPP_ERROR(get_logger(), "MoveGroupInterface not initialized.");
        continue;
      }

      // Use the frame coming from the stub (typically 'world')
      move_group_->setPoseReferenceFrame(goal.header.frame_id);

      move_group_->setStartStateToCurrentState();
      move_group_->setPoseTarget(goal.pose);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto result = move_group_->plan(plan);

      bool ok = (result == moveit::core::MoveItErrorCode::SUCCESS);
      if (!ok)
      {
        RCLCPP_WARN(get_logger(), "Planning failed to goal in frame '%s'.", goal.header.frame_id.c_str());
        continue;
      }

      RCLCPP_INFO(get_logger(), "Plan OK. Executing...");
      auto exec_result = move_group_->execute(plan);
      if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_WARN(get_logger(), "Execution failed.");
      }
      else
      {
        RCLCPP_INFO(get_logger(), "Execution complete.");
      }
    }
  }

  std::string planning_group_;
  std::string pose_reference_frame_;
  double vel_scaling_{0.2};
  double acc_scaling_{0.2};
  double planning_time_sec_{5.0};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  std::mutex mtx_;
  std::condition_variable cv_;
  std::optional<geometry_msgs::msg::PoseStamped> latest_goal_;
  bool stop_{false};
  std::thread worker_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ArmExecutor>();
  node->start();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}