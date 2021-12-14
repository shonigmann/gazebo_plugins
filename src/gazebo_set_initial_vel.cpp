/* 
 *
 */

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class SetInitialVel: public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      gzlog << "Setting Initial Velocity" << std::endl;
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      		std::bind(&SetInitialVel::OnUpdate, this));
      
      // Get initial state from SDF
	    if (!_sdf->HasElement("v_i") && !_sdf->HasElement("w_i"))
			{
				gzerr << "[set_initial_vel] SDF must specify <v_i> or <w_i>." << std::endl;
				return;
			}

	    if (_sdf->HasElement("v_i")) {
	      v_i = _sdf->GetElement("v_i")->Get<ignition::math::Vector3d>();
				//gzlog << "[set_initial_vel] Setting " << v_i << " for v_i" << std::endl;
	    }
	    
	    if (_sdf->HasElement("w_i")) {
	      w_i = _sdf->GetElement("w_i")->Get<ignition::math::Vector3d>();
				//gzlog << "[set_initial_vel] Setting " << w_i << " for w_i" << std::endl;
	    }
	    
	    return;
    }

    
    // Called by the world update start event
    public: void OnUpdate()
    {	    
      // Apply initial velocity
			if (!vel_set) {
				gzlog << "[set_initial_vel] Setting " << v_i << " for v_i; setting " << w_i << " for w_i" << std::endl;
		    this->model->SetLinearVel(v_i);
		    this->model->SetAngularVel(w_i);
		    vel_set = true;
			}
      
      return;
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: 
      event::ConnectionPtr updateConnection;
			ignition::math::Vector3d v_i = ignition::math::Vector3d(0.0, 0.0, 0.0);
			ignition::math::Vector3d w_i = ignition::math::Vector3d(0.0, 0.0, 0.0);
			bool vel_set = false;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SetInitialVel)
}
