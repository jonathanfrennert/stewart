#ifndef _Ball2_PLUGIN_HH_
#define _Ball2_PLUGIN_HH_

#include <vector>
#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32MultiArray.h"


namespace gazebo
{
    /// \brief A plugin to control a ball.
    class Ball2Plugin : public ModelPlugin
    {
      /// \brief Constructor
      public: Ball2Plugin() {}

        /// \brief The load function is called by Gazebo when the plugin is
        /// inserted into simulation
        /// \param[in] _model A pointer to the model that this plugin is
        /// attached to.
        /// \param[in] _sdf A pointer to the plugin's SDF element.
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
          // Store the model pointer for convenience.
          this->model = _model;

          // ----------- ROS TRANSPORT -----------

          // Initialize ROS, if it has not already been initialized.
          if (!ros::isInitialized())
          {
           int argc = 0;
           char **argv = NULL;
           ros::init(argc, argv, "gazebo_client",
               ros::init_options::NoSigintHandler);
          }
          std::cout << "Ball2 PLUGIN: ROS is initialized" << "\n\n";

          // Create our ROS node. This acts in a similar manner to
          // the Gazebo node
          this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

          // Create a named topic for the position and velocity, and subscribe to it.
          ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
              "/" + this->model->GetName() + "/position_velocity_cmd",
              100,
              boost::bind(&Ball2Plugin::OnRosMsg, this, _1),
              ros::VoidPtr(), &this->rosQueue);
          this->rosSub = this->rosNode->subscribe(so);

          // Spin up the queue helper thread.
          this->rosQueueThread =
           std::thread(std::bind(&Ball2Plugin::QueueThread, this));

        }


        /// \brief Handle an incoming message from ROS
        /// \param[in] _msg A float value that is used to set the position and
        /// velocity of the stewart platform
        void OnRosMsg(const std_msgs::Float32MultiArray::ConstPtr& msg)
        {
          this->setPosition(msg);
          this->setVelocity(msg);
        }


        /// \brief Set the position of the ball
        void setPosition(const std_msgs::Float32MultiArray::ConstPtr& msg)
        {
          model->GetLink("ball_link")->SetRelativePose({msg->data[0], msg->data[1], msg->data[2], 0.0, 0.0, 0.0});
        }


        /// \brief Set the velocity of the ball
        void setVelocity(const std_msgs::Float32MultiArray::ConstPtr& msg)
        {
            model->GetLink("ball_link")->SetLinearVel({msg->data[3], msg->data[4], msg->data[5]});
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
    GZ_REGISTER_MODEL_PLUGIN(Ball2Plugin)
}

#endif // _Ball2_PLUGIN_HH_
