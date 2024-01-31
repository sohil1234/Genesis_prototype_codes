#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <rocket_vis/RocketData.h>
#include <geometry_msgs/Vector3.h>

namespace gazebo
{
  class RocketPlugin : public ModelPlugin
  {
  private:
    physics::ModelPtr model;
    physics::LinkPtr link;
    event::ConnectionPtr updateConnection;
    ros::NodeHandle nh;
    ros::Subscriber rocketDataSub;
    ignition::math::Vector3d velocity; // Use ignition math for velocity
    ignition::math::Quaterniond orientation; // Quaternion for orientation

  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      model = _model;
      link = model->GetLink("base_link"); // Modify the link name as needed
      velocity = ignition::math::Vector3d::Zero;
      orientation = ignition::math::Quaterniond::Identity;

      // Initialize ROS (Assume ROS is initialized externally)
      ros::NodeHandle nh;
      rocketDataSub = nh.subscribe("rocket_data", 1, &RocketPlugin::OnRocketData, this);

      // Listen to the update event
      updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&RocketPlugin::OnUpdate, this));
    }

    void OnRocketData(const rocket_vis::RocketDataConstPtr &msg)
    {
      // Update velocity and orientation based on received RocketData
      velocity = ignition::math::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
      ROS_INFO("Received velocity: X=%f, Y=%f, Z=%f",
           msg->velocity.x, msg->velocity.y, msg->velocity.z);

      // Update orientation (pitch, yaw, and roll)
      ignition::math::Quaterniond desiredOrientation;
      desiredOrientation.Euler(msg->angular_position.x, msg->angular_position.y, msg->angular_position.z);
      orientation = desiredOrientation;
      ROS_INFO("Received orientation (Euler): Roll=%f, Pitch=%f, Yaw=%f",
           msg->angular_position.x, msg->angular_position.y, msg->angular_position.z);
    }

    void OnUpdate()
    {
      // Check for null pointers before using them
      if (!model || !link)
      {
        ROS_ERROR("Model or link is null. Plugin not properly initialized.");
        return;
      }

      // Apply velocity control
      link->SetLinearVel(velocity);

      // Apply orientation control
      link->SetWorldPose(ignition::math::Pose3d(link->WorldPose().Pos(), orientation));
    }
  };
  GZ_REGISTER_MODEL_PLUGIN(RocketPlugin)
}

