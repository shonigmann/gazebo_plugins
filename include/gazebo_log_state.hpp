#ifndef GAZEBO_PLUGINS__GAZEBO_LOG_STATE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_LOG_STATE_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboLogStatePrivate;

/// Log model information to a specified log file path
/**
  \details If the joint contains more than one axis, only the state of the first axis is reported.
  Example Usage:
  \code{.xml}
    <plugin name="gazebo_log_state"
        filename="libgazebo_log_state.so">
      <!-- File path -->
      <file_path>~/data/my_data.csv</file_path>
      <log_description>test conditions 1, 2, 3</log_description>

      <!-- Log rate in Hertz; Set <= 0 for as fast as possible -->
      <update_rate>2</update_rate>

      <!-- Optional: Name of links in the model whose states will be logged. -->
      <link_name>base_link</link_name>
      <link_name>wheel_link</link_name>

      <!-- Optional: Name of joints in the model whose states will be logged. -->
      <joint_name>left_wheel</joint_name>
      <joint_name>right_wheel</joint_name>
      <joint_name>elbow</joint_name>
      <joint_name>shoulder</joint_name>

      <!-- Toggle which data to log -->
      <log_sim_time>1</log_sim_time>
      <log_real_time>1</log_real_time>

      <log_pos>1</log_pos>
      <log_vel>0</log_vel>
      <log_acc>0</log_acc>
      <log_joint_force>0</log_joint_force>
      <log_joint_torque>0</log_joint_torque>

    </plugin>
  \endcode
*/
class GazeboLogState : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboLogState();

  /// Destructor
  ~GazeboLogState();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  /// Callback to be called at every simulation iteration.
  /// Private data pointer
  std::unique_ptr<GazeboLogStatePrivate> impl_;
};
}  // namespace gazebo_plugins
#endif  // GAZEBO_PLUGINS__GAZEBO_LOG_STATE_HPP_
