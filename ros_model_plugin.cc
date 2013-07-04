#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h" // use this for position and velocity for now

namespace gazebo
{   
  class ROSModelPlugin : public ModelPlugin
  {

    public: ROSModelPlugin()
    {

      // Start up ROS
      std::string name = "pendulum";
      int argc = 0;
      ros::init(argc, NULL, name);

    }
    public: ~ROSModelPlugin()
    {
      delete this->node;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // ROS Nodehandle
      this->node = new ros::NodeHandle("~");

      this->joint = this->model->GetJoint( "pendulum_joint" );

      // ROS Subscriber
      this->sub = this->node->subscribe<std_msgs::Float64>( "torque", 1000, 
                                                            &ROSModelPlugin::ROSCallback, this );

      // ROS Publisher
      this->pub = this->node->advertise<geometry_msgs::Vector3>( "motion", 1000 );

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ROSModelPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // There is no built-in acceleration function, so need to calculate it manually
      common::Time current_time = this->model->GetWorld()->GetSimTime();
      double dt = current_time.Double() - this->last_update_time.Double();
      this->last_update_time = current_time;
      this->joint_acceleration = ( this->joint->GetVelocity(0) - this->joint_velocity ) / dt;
      this->joint_position = this->joint->GetAngle(0).Radian();
      this->joint_velocity = this->joint->GetVelocity(0);
      this->position_msg.x = this->joint_position;
      this->position_msg.y = this->joint_velocity;
      this->position_msg.z = this->joint_acceleration;
      this->pub.publish( this->position_msg );
      ros::spinOnce();
    }

    void ROSCallback(const std_msgs::Float64::ConstPtr& msg)
    {
      this->joint->SetForce(0, msg->data);
    }

    private:
      physics::JointPtr joint;
      double joint_position;
      double joint_velocity;
      double joint_acceleration;
      geometry_msgs::Vector3 position_msg;
      common::Time last_update_time;


    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    private: ros::NodeHandle* node;

    // ROS Subscriber
    ros::Subscriber sub;
    
    // ROS Publisher
    ros::Publisher pub;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ROSModelPlugin)
}
