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
    ignition::math::Vector3d velocity;
    ignition::math::Quaterniond orientation;

  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      model = _model;
      link = model->GetLink("base_link");
      velocity = ignition::math::Vector3d::Zero;
      orientation = ignition::math::Quaterniond::Identity;

      // Initialize ROS (Assume ROS is initialized externally)
      ros::NodeHandle nh;
      rocketDataSub = nh.subscribe("rocket_data", 1, &RocketPlugin::OnRocketData, this);

      // Listen to the update event at a reduced rate (e.g., every 0.01 seconds)
      updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&RocketPlugin::OnUpdate, this));
    }

    void OnRocketData(const rocket_vis::RocketDataConstPtr &msg)
    {
      velocity = ignition::math::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
      orientation.Euler(msg->angular_position.x, msg->angular_position.y, msg->angular_position.z);
    }

    void OnUpdate()
    {
      if (!model || !link)
      {
        ROS_ERROR("Model or link is null. Plugin not properly initialized.");
        return;
      }

      // Apply velocity control
      link->SetLinearVel(velocity);

      // Apply orientation control smoothly
      double step = 0.01; // Adjust the step size for smoothness (e.g., 0.01 seconds)
      ignition::math::Quaterniond currentOrientation = link->WorldPose().Rot();
      ignition::math::Quaterniond newOrientation = ignition::math::Quaterniond::Slerp(step, currentOrientation, orientation);
      link->SetWorldPose(ignition::math::Pose3d(link->WorldPose().Pos(), newOrientation));
    }
  };
  GZ_REGISTER_MODEL_PLUGIN(RocketPlugin)
}

