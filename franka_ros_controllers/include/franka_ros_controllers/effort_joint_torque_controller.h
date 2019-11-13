#pragma once

#include <memory>
#include <string>
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <franka_core_msgs/JointCommand.h>
#include <franka_core_msgs/JointControllerStates.h>
#include <franka_core_msgs/JointLimits.h>

#include <franka_hw/trigger_rate.h>
#include <realtime_tools/realtime_publisher.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <mutex>

namespace franka_ros_controllers {

class EffortJointTorqueController : public controller_interface::MultiInterfaceController<
                                            hardware_interface::EffortJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  static std::array<double, 7> saturateTorqueRate(
      const std::array<double, 7>& tau_d_calculated);  // NOLINT (readability-identifier-naming)

  std::vector<hardware_interface::JointHandle> joint_handles_;

  static constexpr double kDeltaTauMax{1.0};

  std::array<double, 7> jnt_cmd_{};
  std::array<double, 7> prev_jnt_cmd_{};


  franka_hw::TriggerRate rate_trigger_{1.0};
  std::array<double, 7> last_tau_d_{};

  ros::Subscriber desired_joints_subscriber_;

  franka_core_msgs::JointLimits joint_limits_;

  franka_hw::TriggerRate trigger_publish_;
  realtime_tools::RealtimePublisher<franka_core_msgs::JointControllerStates> publisher_controller_states_;

  bool checkTorqueLimits(std::vector<double> torques);

  void jointCmdCallback(const franka_core_msgs::JointCommandConstPtr& msg);
};

}  // namespace franka_ros_controllers