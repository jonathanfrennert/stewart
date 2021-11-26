#ifndef _STEWART_PLUGIN_HH_
#define _STEWART_PLUGIN_HH_

#include <vector>
#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
    /// \brief A plugin to control a Stewart Platform.
    class StewartPlugin : public ModelPlugin
    {
      /// \brief Constructor
      public: StewartPlugin() {}

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Safety check
            if (_model->GetJointCount() == 0)
            {
              std::cerr << "Invalid joint count, Stewart plugin not loaded\n";
              return;
            }
            else
              std::cout << "\nThe Stewart plugin is attached to model[" << _model->GetName() << "]\n";

            // Store the model pointer for convenience.
            this->model = _model;

            // Get the first joint. We are making an assumption about the model
            // having one joint that is the rotational joint.
            this->joints = _model->GetJoints();

            // Setup a P-controller, with a gain of 0.1.
            this->pid = common::PID(0.1, 0, 0);


            // ----------- POSITION -----------


            // Apply the P-controller to the position of the joints.
            /*
            for (const auto& joint: joints)
              this->model->GetJointController()->SetPositionPID(
                  joint->GetScopedName(), this->pid);
            */


            // ----------- VELOCITY -----------

            // Apply the P-controller to the velocity of the joints.
            for (const auto& joint: this->joints)
              this->model->GetJointController()->SetVelocityPID(
                  joint->GetScopedName(), this->pid);

            this->SetVelocity(0);


            // ----------- TRANSPORT -----------

            // Create the node
            this->node = transport::NodePtr(new transport::Node());
            #if GAZEBO_MAJOR_VERSION < 8
            this->node->Init(this->model->GetWorld()->GetName());
            #else
            this->node->Init(this->model->GetWorld()->Name());
            #endif

            // Create a topic name
            std::string topicName = "~/" + _model->GetName() + "/vel_cmd";

            // Subscribe to the topic, and register a callback
            this->sub = this->node->Subscribe(topicName,
               &StewartPlugin::OnMsg, this);
        }


        /// \brief Set the position of all the joints
        /// \param[in] _pos New target position
        public: void SetPosition(const double &_pos)
        {
          // Set the joint's target position.
          for (const auto& joint: this->joints)
            this->model->GetJointController()->SetPositionPID(
                joint->GetScopedName(), _pos);
        }


        /// \brief Set the velocity of all the joints
        /// \param[in] _vel New target velocity
        public: void SetVelocity(const double &_vel)
        {
          // Set the joint's target velocity.
          for (const auto& joint: this->joints)
            this->model->GetJointController()->SetVelocityPID(
                joint->GetScopedName(), _vel);
        }


        /// \brief Handle incoming message
        /// \param[in] _msg Repurpose a vector3 message. This function will
        /// only use the x component.
        private: void OnMsg(ConstVector3dPtr &_msg)
        {
          this->SetVelocity(_msg->x());
        }


        // ----------- MODEL -----------

        /// \brief Pointer to the model.
        private: physics::ModelPtr model;

        /// \brief Pointer to the joints.
        private: std::vector<physics::JointPtr> joints;

        /// \brief A PID controller for the joint.
        private: common::PID pid;


        // ----------- GAZEBO TRANSPORT -----------

        /// \brief A node used for transport
        private: transport::NodePtr node;

        /// \brief A subscriber to a named topic.
        private: transport::SubscriberPtr sub;
    };


    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(StewartPlugin)
}

#endif // _STEWART_PLUGIN_HH_