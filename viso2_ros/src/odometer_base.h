
#ifndef ODOMETER_BASE_H_
#define ODOMETER_BASE_H_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <std_srvs/srv/empty.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>



#include <tf2_helper.h>

namespace viso2_ros
{

/**
 * Base class for odometers, handles tf's, odometry and pose
 * publishing. This can be used as base for any incremental pose estimating
 * sensor. Sensors that measure velocities cannot be used.
 */
class OdometerBase : public rclcpp::Node
{
private:

  // publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;

  // tf related
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string sensor_frame_id_;
  std::string odom_frame_id_;
  std::string base_link_frame_id_;
  bool publish_tf_;
  bool invert_tf_;

  // the current integrated camera pose
  tf2::Transform integrated_pose_;


  // covariances
  std::array<double, 36> pose_covariance_;
  std::array<double, 36> twist_covariance_;

public:
  //mfc
  // timestamp of the last update
  rclcpp::Time last_update_time_;

  OdometerBase(std::string name ) : Node(name) 
  {
    // Read local parameters
    odom_frame_id_ = this->declare_parameter("odom_frame_id", "odom");
    base_link_frame_id_ = this->declare_parameter("base_link_frame_id", "base_link");
    sensor_frame_id_ = this->declare_parameter("sensor_frame_id", "camera");
    publish_tf_ = this->declare_parameter("publish_tf", true);
    invert_tf_ = this->declare_parameter("invert_tf", false);

    RCLCPP_INFO(this->get_logger(), "Basic Odometer Settings: odom_frame_id = %s \n base_link_frame_id = %s \n publish_tf = %s \n invert_tf = %s", odom_frame_id_.c_str(), base_link_frame_id_.c_str(), (publish_tf_?"true":"false"), (invert_tf_?"true":"false"));

    // advertise
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 1);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 1);
    
    reset_service_ = this->create_service<std_srvs::srv::Empty>("reset_pose", std::bind(&OdometerBase::resetPose, this, std::placeholders::_1, std::placeholders::_2));

    integrated_pose_.setIdentity();
    last_update_time_ = this->get_clock()->now();

    pose_covariance_.fill(0.0);
    twist_covariance_.fill(0.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  }

protected:

  void resetPose(const std::shared_ptr<std_srvs::srv::Empty::Request>, const std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    integrated_pose_.setIdentity();
  }

  void setSensorFrameId(const std::string& frame_id)
  {
    sensor_frame_id_ = frame_id;
  }

  std::string getSensorFrameId() const
  {
    return sensor_frame_id_;
  }

  void setPoseCovariance(const std::array<double, 36>& pose_covariance)
  {
    pose_covariance_ = pose_covariance;
  }

  void setTwistCovariance(const std::array<double, 36>& twist_covariance)
  {
    twist_covariance_ = twist_covariance;
  }

  void integrateAndPublish(const tf2::Transform& delta_transform, const rclcpp::Time& timestamp)
  {
    RCLCPP_INFO(this->get_logger(), "[odometer] last timestamp is (%3.3f) sec ", (timestamp).nanoseconds()/1e9  );

    if (sensor_frame_id_.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "[odometer] update called with unknown sensor frame id!");
      return;
    }
    if (timestamp < last_update_time_)
    {
      RCLCPP_WARN(this->get_logger(), "[odometer] saw negative time change (%3.2f s) in incoming sensor data, resetting pose.", (timestamp - last_update_time_).nanoseconds()/1e9  );
      integrated_pose_.setIdentity();
    }
    integrated_pose_ *= delta_transform;

    // transform integrated pose to base frame
    geometry_msgs::msg::TransformStamped base_to_sensor;
    tf2::Stamped<tf2::Transform> base_to_sensor_tf;

    tf2::TimePoint time_point = tf2::timeFromSec(timestamp.seconds());
    std::string error_msg;
    bool is_transformed = false; 

    // first try exact transform
    if (tf_buffer_->canTransform(base_link_frame_id_, sensor_frame_id_, time_point, &error_msg))
    {
      base_to_sensor = tf_buffer_->lookupTransform(
          base_link_frame_id_,
          sensor_frame_id_,
          time_point);
    tf2::fromMsg(base_to_sensor, base_to_sensor_tf);
    RCLCPP_DEBUG(this->get_logger(), "Transform successful");
    is_transformed = true;
    }

    // ok try latest
    if (!is_transformed)
    {
      RCLCPP_WARN(this->get_logger(), "1. The tf from '%s' to '%s' at time '%3.3f' does not seem to be available, "
                              "will try last available!",
                              base_link_frame_id_.c_str(),
                              sensor_frame_id_.c_str(),
                              timestamp.seconds());
      RCLCPP_DEBUG(this->get_logger(), "Transform error: %s", error_msg.c_str());

      try
      {
      base_to_sensor = tf_buffer_->lookupTransform(
            base_link_frame_id_,
            sensor_frame_id_,
            tf2::TimePointZero);
      tf2::fromMsg(base_to_sensor, base_to_sensor_tf);
      RCLCPP_DEBUG(this->get_logger(), "Transform successful");
      is_transformed = true;
      }
      catch (tf2::TransformException ex)
      {
      }

    }

    if (!is_transformed)
    {
      RCLCPP_WARN(this->get_logger(), "2. The tf from '%s' to '%s' at time '%3.3f' does not seem to be available, "
                              "will assume identity!",
                              base_link_frame_id_.c_str(),
                              sensor_frame_id_.c_str(),
                              tf2_ros::toRclcpp(tf2::TimePointZero).seconds());
      RCLCPP_DEBUG(this->get_logger(), "Transform error: %s", error_msg.c_str());

      tf2::fromMsg(base_to_sensor, base_to_sensor_tf); 
      base_to_sensor_tf.setIdentity();
    }


    tf2::Transform base_transform = base_to_sensor_tf * integrated_pose_ * base_to_sensor_tf.inverse();

    nav_msgs::msg::Odometry odometry_msg;
    odometry_msg.header.stamp = timestamp;
    odometry_msg.header.frame_id = odom_frame_id_;
    odometry_msg.child_frame_id = base_link_frame_id_;

    // TODO use tf2 solution instead of setting explicitly
    odometry_msg.pose.pose.orientation.x = base_transform.getRotation().getX();
    odometry_msg.pose.pose.orientation.y = base_transform.getRotation().getY();
    odometry_msg.pose.pose.orientation.z = base_transform.getRotation().getZ();
    odometry_msg.pose.pose.orientation.w = base_transform.getRotation().getW();

    odometry_msg.pose.pose.position.x = base_transform.getOrigin().getX();
    odometry_msg.pose.pose.position.y = base_transform.getOrigin().getY();
    odometry_msg.pose.pose.position.z = base_transform.getOrigin().getZ();


    //tf2::convert(base_transform, odometry_msg.pose.pose);

    // calculate twist (not possible for first run as no delta_t can be computed)
    tf2::Transform delta_base_transform = base_to_sensor_tf * delta_transform * base_to_sensor_tf.inverse();
    if (last_update_time_.nanoseconds() != 0) 
    {
      double delta_t = (timestamp - last_update_time_).seconds();
      if (delta_t)
      {
        odometry_msg.twist.twist.linear.x = delta_base_transform.getOrigin().getX() / delta_t;
        odometry_msg.twist.twist.linear.y = delta_base_transform.getOrigin().getY() / delta_t;
        odometry_msg.twist.twist.linear.z = delta_base_transform.getOrigin().getZ() / delta_t;
        tf2::Quaternion delta_rot = delta_base_transform.getRotation();
        tf2Scalar angle = delta_rot.getAngle();
        tf2::Vector3 axis = delta_rot.getAxis();
        tf2::Vector3 angular_twist = axis * angle / delta_t;
        odometry_msg.twist.twist.angular.x = angular_twist.x();
        odometry_msg.twist.twist.angular.y = angular_twist.y();
        odometry_msg.twist.twist.angular.z = angular_twist.z();
      }
    }

    odometry_msg.pose.covariance = pose_covariance_;
    odometry_msg.twist.covariance = twist_covariance_;
    odom_pub_->publish(odometry_msg);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = odometry_msg.header.stamp;
    pose_msg.header.frame_id = odometry_msg.header.frame_id;
    pose_msg.pose = odometry_msg.pose.pose;

    pose_pub_->publish(pose_msg);

    geometry_msgs::msg::TransformStamped tf_stamped_msg;
    tf_stamped_msg.header.stamp = timestamp;
    tf_stamped_msg.header.frame_id = odom_frame_id_;
    tf_stamped_msg.child_frame_id = base_link_frame_id_;


    tf2::Stamped<tf2::Transform> base_transform_stamped;
    geometry_msgs::msg::TransformStamped base_transform_stamped_msg;
    
    if (publish_tf_)
    {
      if (invert_tf_)
      {
        base_transform_stamped.setData(base_transform.inverse());
        base_transform_stamped_msg = tf2::toMsg(base_transform_stamped);
        tf_stamped_msg.transform = base_transform_stamped_msg.transform;
        tf_broadcaster_->sendTransform(tf_stamped_msg);
      }
      else
      {
        base_transform_stamped.setData(base_transform);
        base_transform_stamped_msg = tf2::toMsg(base_transform_stamped);
        tf_stamped_msg.transform = base_transform_stamped_msg.transform;
        tf_broadcaster_->sendTransform(tf_stamped_msg);
      }
    }

    last_update_time_ = timestamp;
    RCLCPP_INFO(this->get_logger(), "[odometer] last update time is (%3.3f) sec ", (last_update_time_).nanoseconds()/1e9  );
  }

};

} // end of namespace

#endif

