#ifndef _STEWART_PLUGIN_HH_
#define _STEWART_PLUGIN_HH_

#include "ik.h"

#include "eigen3/Eigen/Core"

#include <vector>
#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
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
              std::cout << "\nThe Stewart plugin is attached to model[" << _model->GetName() << "]\n";

            // Store the model pointer for convenience.
            this->model = _model;

            // Get the first joint. We are making an assumption about the model
            // having one joint that is the rotational joint.
            this->joints = _model->GetJoints();

            // Setup a P-controller, with a gain of 0.1.
            //this->pid = common::PID(10, 0, 10);
            this->pid = common::PID(10000.0, 0.1, 100.0);

            // ----------- POSITION -----------

            // Apply the P-controller to the position of the joints.
            for (const auto& joint: this->joints)
            {
              // set joint to PID controller
              this->model->GetJointController()->SetPositionPID(
                  joint->GetScopedName(), this->pid);
            }


            // ----------- VELOCITY -----------

            // Apply the P-controller to the velocity of the joints.
            for (const auto& joint: this->joints)
            {
              this->model->GetJointController()->SetVelocityPID(
                  joint->GetScopedName(), this->pid);
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
            std::cout << "ROS is initialized" << "\n";

            // Create our ROS node. This acts in a similar manner to
            // the Gazebo node
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            // Create a named topic, and subscribe to it.
            ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
                "/" + this->model->GetName() + "/position_cmd",
                100,
                boost::bind(&StewartPlugin::setPosition, this, _1),
                ros::VoidPtr(), &this->rosQueue);
            this->rosSub = this->rosNode->subscribe(so);

           // Spin up the queue helper thread.
           this->rosQueueThread =
             std::thread(std::bind(&StewartPlugin::QueueThread, this));

           // Listen to the update event. This event is broadcast every
           // simulation iteration.
           this->updateConnection = event::Events::ConnectWorldUpdateBegin(
               std::bind(&StewartPlugin::OnUpdate, this));

           this->startTime = common::Time::GetWallTime().Double();
        }


        // Called by the world update start event
        public: void OnUpdate()
        {
          double timeNow = common::Time::GetWallTime().Double();
          double elapsedTime = timeNow - this->startTime;

          //printf("Elapsed time is %f\n", elapsedTime);
        }


        /// \brief Set the velocity of all the joints
        /// \param[in] _vel New target velocity
        void setPosition(const std_msgs::Float32MultiArray::ConstPtr& msg)
        {
            Eigen::Matrix<float, 6, 1> goalJointPos = ik(msg);

            auto joints_it = std::begin(this->joints);
            for (int i = 0; i < 5; i++)
                this->model->GetJointController()->SetPositionTarget((*joints_it++)->GetScopedName(), goalJointPos[i]);
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


        // ----------- WORLD -----------

        private: double startTime;

        // ----------- MODEL -----------

        /// \brief Pointer to the model.
        private: physics::ModelPtr model;

        /// \brief Pointer to the joints.
        private: std::vector<physics::JointPtr> joints;

        /// \brief A PID controller for the joint.
        private: common::PID pid;


        // ----------- GAZEBO TRANSPORT ---------

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;


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
