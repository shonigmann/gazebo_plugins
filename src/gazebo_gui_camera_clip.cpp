#include <functional>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>


namespace gazebo
{
  class SystemGUI : public SystemPlugin
  {
    /////////////////////////////////////////////
    /// \brief Destructor
    public: virtual ~SystemGUI()
    {
      this->connections.clear();
      this->userCam.reset();
    }

    /////////////////////////////////////////////
    /// \brief Called after the plugin has been constructed.
    public: void Load(int /*_argc*/, char ** /*_argv*/)
    {
        this->connections.push_back(
          event::Events::ConnectPreRender(
            std::bind(&SystemGUI::Update, this)));
    }

    /////////////////////////////////////////////
    // \brief Called once after Load
    private: void Init()
    {
      std::cout << "Init\n";
    }

    /////////////////////////////////////////////
    /// \brief Called every PreRender event. See the Load function.
    private: void Update()
    {
      if (!this->userCam)
      {
        // Get a pointer to the active user camera
        this->userCam = gui::get_active_camera();

        // Set Clip Distances
        const float near = 0.1;
        const float far = 1000000.0;
        gzdbg << "Setting GUI Camera Clip Distances\n";
        this->userCam->SetClipDist(near, far);
      }
    }

    /// Pointer the user camera.
    private: rendering::UserCameraPtr userCam;

    /// All the event connections.
    private: std::vector<event::ConnectionPtr> connections;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(SystemGUI)
}
