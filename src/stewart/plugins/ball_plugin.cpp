#ifndef _BALL_PLUGIN_HH_
#define _BALL_PLUGIN_HH_

#include <vector>
#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>


namespace gazebo
{
    /// \brief A plugin to control a ball.
    class BallPlugin : public ModelPlugin
    {
      /// \brief Constructor
      public: BallPlugin() {}

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Store the model pointer for convenience.
            this->model = _model;

            model->GetLink("ball_link")->SetLinearVel({-1.1, -1.1, 4.8});
        }


        // ----------- MODEL -----------

        /// \brief Pointer to the model.
        private: physics::ModelPtr model;

        /// \brief Pointer to the joints.
        private: std::vector<physics::JointPtr> joints;
    };


    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(BallPlugin)
}

#endif // _BALL_PLUGIN_HH_
