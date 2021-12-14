// #include <functional>
#include <fstream> 

#include <gazebo/gazebo.hh>

#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <spaceros_gz_plugins/gazebo_log_state.hpp>

namespace gazebo_plugins
{

  class GazeboLogStatePrivate
  {
  public:

    /// Callback to be called at every simulation iteration.
    /// \param[in] info Updated simulation info.
    void OnUpdate(const gazebo::common::UpdateInfo & info);

    /// Joints being tracked.
    std::vector<gazebo::physics::JointPtr> joints_;

    /// Links being tracked.
    std::vector<gazebo::physics::LinkPtr> links_;

    /// Period in seconds
    double update_period_;

    /// Keep last time an update was published
    gazebo::common::Time last_update_time_;

    /// Pointer to the update event connection.
    gazebo::event::ConnectionPtr update_connection_;

    /// Path to output log file
    std::string file_path_;

    /// Whether to log simulated time
    bool log_sim_time_;
    /// Whether to log system time
    bool log_real_time_;
    /// Whether to log link and joint positions
    bool log_pos_;
    /// Whether to log link and joint velocities
    bool log_vel_;
    /// Whether to log link and joint accelerations
    bool log_acc_;
    /// Whether to log external joint wrenches
    bool log_joint_wrench_;
  };


  GazeboLogState::GazeboLogState()
  : impl_(std::make_unique<GazeboLogStatePrivate>())
  {
  }

  GazeboLogState::~GazeboLogState()
  {
  }
  

  void GazeboLogState::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    gzdbg << " Starting get_info Plugin" << std::endl;

    // Check for link or joint elements
    if (!sdf->HasElement("link_name") && !sdf->HasElement("joint_name")) {
        gzdbg << "ERROR: YOU MUST SPECIFY AT LEAST ONE JOINT OR LINK" << std::endl;
        return;
    }

    // get link names 
    if (sdf->HasElement("link_name")) {
      sdf::ElementPtr link_elem = sdf->GetElement("link_name");
      while (link_elem) {
        auto link_name = link_elem->Get<std::string>();
        auto link = model->GetLink(link_name);
        if (!link) {
          gzdbg << "Link " << link_name.c_str() << " does not exist!" << std::endl;
        } else {
          impl_->links_.push_back(link);
          gzdbg << "Going to log link " << link_name.c_str() << std::endl;
        }
        link_elem = link_elem->GetNextElement("link_name");
      }
    }    
    
    // get joint names 
    if (sdf->HasElement("joint_name")) {
      sdf::ElementPtr joint_elem = sdf->GetElement("joint_name");
      while (joint_elem) {
        auto joint_name = joint_elem->Get<std::string>();
        auto joint = model->GetJoint(joint_name);
        if (!joint) {
          gzdbg << "Joint " << joint_name.c_str() << " does not exist!" << std::endl;
        } else {
          impl_->joints_.push_back(joint);
          gzdbg << "Going to log joint " << joint_name.c_str() << std::endl;
        }
        joint_elem = joint_elem->GetNextElement("joint_name");
      }
    }

    // make sure we have at least one joint or link to log data for
    if (impl_->links_.empty() && impl_->joints_.empty()) {
      gzdbg << "No joints or links found" << std::endl;
      return;
    }

    // Whether to log simulated time
    impl_->log_sim_time_ = true;
    if (sdf->HasElement("log_sim_time")) {
      impl_->log_sim_time_ = sdf->GetElement("log_sim_time")->Get<bool>();
    }

    // Whether to log system time
    impl_->log_real_time_ = false;
    if (sdf->HasElement("log_real_time")) {
      impl_->log_real_time_ = sdf->GetElement("log_real_time")->Get<bool>();
    }

    // Whether to log link and joint positions
    impl_->log_pos_ = true;
    if (sdf->HasElement("log_pos")) {
      impl_->log_pos_ = sdf->GetElement("log_pos")->Get<bool>();
    }

    // Whether to log link and joint velocities
    impl_->log_vel_ = false;
    if (sdf->HasElement("log_vel")) {
      impl_->log_vel_ = sdf->GetElement("log_vel")->Get<bool>();
    }

    // Whether to log link and joint accelerations
    impl_->log_acc_ = false;
    if (sdf->HasElement("log_acc")) {
      impl_->log_acc_ = sdf->GetElement("log_acc")->Get<bool>();
    }
    
    // Whether to log external joint wrenches
    impl_->log_joint_wrench_ = false;
    if (sdf->HasElement("log_joint_wrench")) {
      impl_->log_joint_wrench_ = sdf->GetElement("log_joint_wrench")->Get<bool>();
    }

    // Update rate
    double update_rate = 100.0;
    if (!sdf->HasElement("update_rate")) {
      gzdbg << "Missing <update_rate>, defaults to " << update_rate << " (log every step)" << std::endl;
    } else {
      update_rate = sdf->GetElement("update_rate")->Get<double>();
    }
    
    // Path to output log file
    if (!sdf->HasElement("file_path")) {
      gzdbg << "No output file specified. Exiting." << std::endl;
      return;
    } 
    else {
      impl_->file_path_ = sdf->GetElement("file_path")->Get<std::string>();

      // try to append to file:
      std::ofstream outfile;
      outfile.open(impl_->file_path_, std::ios_base::app);
      if (outfile) {    
        if (sdf->HasElement("log_description")) {
          // append description to file if there is one
          outfile << std::endl << sdf->GetElement("log_description")->Get<std::string>() << std::endl;
        }

        // append headers in CSV format; to make compatibility easy, include headers even if the data will be unfilled
        outfile << "sim_time, real_time, ";

        // add headers for links
        for (unsigned int i = 0; i < impl_->links_.size(); ++i) {
          std::string n = impl_->links_[i]->GetName();
          outfile << n << "_px, " << n << "_py, " << n << "_pz, ";
          outfile << n << "_qw, " << n << "_qx, " << n << "_qy, " << n << "_qz, ";
          outfile << n << "_vx, " << n << "_vy, " << n << "_vz, ";
          outfile << n << "_wx, " << n << "_wy, " << n << "_wz, ";
          outfile << n << "_ax, " << n << "_ay, " << n << "_az, ";
          outfile << n << "_alphax, " << n << "_alphay, " << n << "_alphaz, ";
        }

        // add headers for joints
        for (unsigned int i = 0; i < impl_->joints_.size(); ++i) {
          std::string n = impl_->joints_[i]->GetName();
          outfile << n << "_p, " << n << "_v, ";
          outfile << n << "_Fx, " << n << "_Fy, " << n << "_Fz, ";
          outfile << n << "_Tx, " << n << "_Ty, " << n << "_Tz, ";
        }

        outfile << std::endl;
      }
      else {
        gzdbg << "Error opening file '" << impl_->file_path_ << "'. Exiting" << std::endl;
        return;
      }      
    }

    // convert Rate to Period
    if (update_rate > 0.0) {
      impl_->update_period_ = 1.0 / update_rate;
    } else {
      impl_->update_period_ = 0.0;
    }

    impl_->last_update_time_ = model->GetWorld()->SimTime();

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboLogStatePrivate::OnUpdate, impl_.get(), std::placeholders::_1));
  }


  // Called by the world update start event
  void GazeboLogStatePrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    gazebo::common::Time current_sim_time = info.simTime;
    gazebo::common::Time current_real_time = info.realTime;

    // If the world is reset, for example
    if (current_sim_time < last_update_time_) {
      gzdbg << "Negative sim time difference detected" << std::endl;
      last_update_time_ = current_sim_time;
    }

    // Check period
    double seconds_since_last_update = (current_sim_time - last_update_time_).Double();

    if (seconds_since_last_update < update_period_) {
      return;
    }

    // Open Log File
    std::ofstream outfile;
    outfile.open(file_path_, std::ios_base::app);
    if (outfile) {

      // add sim time
      if (log_sim_time_)
        outfile << current_sim_time.Double();
      outfile << ", ";

      // add real time
      if (log_real_time_)
        outfile << current_real_time.Double();
      outfile << ", ";

      // add data for links
      for (unsigned int i = 0; i < links_.size(); ++i) {
        // link Pose
        if (log_pos_){
          ignition::math::Pose3d p = links_[i]->WorldPose();
          outfile << p.Pos()[0] << ", " << p.Pos()[1] << ", " << p.Pos()[2] << ", "; // position
          outfile << p.Rot().W() << ", " << p.Rot().X() << ", " << p.Rot().Y() << ", " << p.Rot().Z() << ", "; // quaternion orientation
        } 
        else {          
          outfile << ", , , , , , , ";
        }

        // link accelerations
        if (log_vel_){
          ignition::math::Vector3d v = links_[i]->WorldLinearVel();
          outfile << v[0] << ", " << v[1] << ", " << v[2] << ", ";

          ignition::math::Vector3d w = links_[i]->WorldAngularVel();
          outfile << w[0] << ", " << w[1] << ", " << w[2] << ", ";
        }
        else {
          outfile << ", , , , , , ";
        }

        // link accelerations
        if (log_acc_){
          ignition::math::Vector3d a = links_[i]->WorldLinearAccel();
          outfile << a[0] << ", " << a[1] << ", " << a[2] << ", ";

          ignition::math::Vector3d alpha = links_[i]->WorldAngularAccel();
          outfile << alpha[0] << ", " << alpha[1] << ", " << alpha[2] << ", ";
        }
        else {
          outfile << ", , , , , , ";
        }
      }

      // add data for joints
      for (unsigned int i = 0; i < joints_.size(); ++i) {
        std::string n = joints_[i]->GetName();
        
        // joint position
        if (log_pos_){
          double position = joints_[i]->Position(0);
          outfile << position;
        }
        outfile << ", ";

        // joint velocity
        if (log_vel_){
          double velocity = joints_[i]->GetVelocity(0);
          outfile << velocity;
        }
        outfile << ", ";

        // joint force
        if (log_joint_wrench_){
          gazebo::physics::JointWrench wrench = joints_[i]->GetForceTorque(0);

          ignition::math::Vector3d F = wrench.body1Force;
          outfile << F[0] << ", " << F[1] << ", " << F[2] << ", ";

          ignition::math::Vector3d T = wrench.body1Torque;
          outfile << T[0] << ", " << T[1] << ", " << T[2] << ", ";
        } 
        else {          
          outfile << ", , , , , , ";
        }
      } 
      
      // end line
      outfile << std::endl;
    }
    else {
      gzdbg << "Error opening file " << file_path_ << ". Exiting. " << std::endl;
      return;
    }
    
    // Update time
    last_update_time_ = current_sim_time;
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GazeboLogState)
}
