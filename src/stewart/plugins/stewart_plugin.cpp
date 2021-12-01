#ifndef _STEWART_PLUGIN_HH_
#define _STEWART_PLUGIN_HH_

#include <vector>
#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32MultiArray.h"


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
              std::cout << "\nSTEWART PLUGIN: The Stewart plugin is attached to model[" << _model->GetName() << "]\n\n";

            // Store the model pointer for convenience.
            this->model = _model;

            // Get the joints.
            this->joints = _model->GetJoints();

            // Setup a position P-controller (some extreme tuning)
            this->posPid = common::PID(1000000, 100000, 50);

            // Setup a velocity P-controller (some less extreme tuning)
            this->velPid = common::PID(10, 1, 0.01);

            // ----------- POSITION -----------

            // Apply the P-controller to the position of the joints.
            for (const auto& joint: this->joints)
            {
              this->model->GetJointController()->SetPositionPID(
                  joint->GetScopedName(), this->posPid);
            }


            // ----------- VELOCITY -----------

            // Apply the P-controller to the velocity of the joints.
            for (const auto& joint: this->joints)
            {
              this->model->GetJointController()->SetVelocityPID(
                  joint->GetScopedName(), this->velPid);
            }


            // ----------- ROS TRANSPORT -----------

            // Initialize ROS, if it has not already been initialized.
            if (!ros::isInitialized())
            {
             int argc = 0;
             char **argv = NULL;
             ros::init(argc, argv, "gazebo_client",
                 ros::init_options::NoSigintHandler);
            }
            std::cout << "STEWART PLUGIN: ROS is initialized" << "\n\n";

            // Create our ROS node. This acts in a similar manner to
            // the Gazebo node
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            // Create a named topic for the position and velocity, and subscribe to it.
            ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
                "/" + this->model->GetName() + "/position_velocity_cmd",
                100,
                boost::bind(&StewartPlugin::OnRosMsg, this, _1),
                ros::VoidPtr(), &this->rosQueue);
            this->rosSub = this->rosNode->subscribe(so);

           // Spin up the queue helper thread.
           this->rosQueueThread =
             std::thread(std::bind(&StewartPlugin::QueueThread, this));
        }

        /// \brief Handle an incoming message from ROS
        /// \param[in] _msg A float value that is used to set the position and
        /// velocity of the stewart platform
        void OnRosMsg(const std_msgs::Float32MultiArray::ConstPtr& msg)
        {
          this->setPosition(msg);
          this->setVelocity(msg);
        }

        /// \brief Set the position of all the joints
        /// \param[in] _vel New target position
        void setPosition(const std_msgs::Float32MultiArray::ConstPtr& msg)
        {
            auto joints_it = std::begin(this->joints);
            for (int i = 0; i < 5; i++)
                this->model->GetJointController()->SetPositionTarget((*joints_it++)->GetScopedName(), msg->data[i]);
        }


        /// \brief Set the velocity of all the joints
        /// \param[in] _vel New target velocity
        void setVelocity(const std_msgs::Float32MultiArray::ConstPtr& msg)
        {
            auto joints_it = std::begin(this->joints);
            for (int i = 6; i < 11; i++)
                this->model->GetJointController()->SetVelocityTarget((*joints_it++)->GetScopedName(), msg->data[i]);
        }


        /// \brief ROS helper function that processes messages
        private: void QueueThread()
        {
          static const double timeout = 0.01;
          while (this->rosNode->ok())
          {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
          }
        }


        // ----------- MODEL -----------

        /// \brief Pointer to the model.
        private: physics::ModelPtr model;

        /// \brief Pointer to the joints.
        private: std::vector<physics::JointPtr> joints;

        /// \brief A PID controller for the position of the joint.
        private: common::PID posPid;

        /// \brief A PID controller for the velocity of the joint.
        private: common::PID velPid;


        // ----------- ROS TRANSPORT -----------

        /// \brief A node use for ROS transport
        private: std::unique_ptr<ros::NodeHandle> rosNode;

        /// \brief A ROS subscriber
        private: ros::Subscriber rosSub;

        /// \brief A ROS callbackqueue that helps process messages
        private: ros::CallbackQueue rosQueue;

        /// \brief A thread the keeps running the rosQueue
        private: std::thread rosQueueThread;
    };


    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(StewartPlugin)
}

#endif // _STEWART_PLUGIN_HH_
