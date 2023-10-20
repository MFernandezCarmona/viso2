#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <viso_mono.h>

#include <viso2_ros/msg/viso_info.hpp>

#include "odometer_base.h"
#include "odometry_params.h"

namespace viso2_ros
{

class MonoOdometer : public OdometerBase
{
private:

  std::shared_ptr<VisualOdometryMono> visual_odometer_;
  VisualOdometryMono::parameters visual_odometer_params_;

  image_transport::CameraSubscriber camera_sub_;
  
  rclcpp::Publisher<viso2_ros::msg::VisoInfo>::SharedPtr info_pub_;

  bool replace_;

public:
  MonoOdometer() :
  OdometerBase("MonoOdometerNode"),
  replace_(false)
  {
    // Read local parameters
    std::string transport = this->declare_parameter("transport", "raw");
    rclcpp::Node* parent = this;
    odometry_params::loadParams(parent, visual_odometer_params_);
    
    camera_sub_ = image_transport::create_camera_subscription(this, "image", std::bind(&MonoOdometer::imageCallback, this, std::placeholders::_1, std::placeholders::_2), transport);

    info_pub_ = this->create_publisher<viso2_ros::msg::VisoInfo>("info", 1); 

  }

protected:
  void imageCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
  {
    auto start_time = this->get_clock()->now();

    //mfc
    if (last_update_time_ > image_msg->header.stamp) {
      RCLCPP_INFO(this->get_logger(), "Ignoring out of secuence image");
      return;
    }

    bool first_run = false;
    // create odometer if not exists
    if (!visual_odometer_)
    {
      first_run = true;
      // read calibration info from camera info message
      // to fill remaining odometer parameters
      image_geometry::PinholeCameraModel model;
      model.fromCameraInfo(info_msg);
      visual_odometer_params_.calib.f  = model.fx();
      visual_odometer_params_.calib.cu = model.cx();
      visual_odometer_params_.calib.cv = model.cy();
      visual_odometer_.reset(new VisualOdometryMono(visual_odometer_params_));
      if (image_msg->header.frame_id != "") {
        setSensorFrameId(image_msg->header.frame_id);
      }
      RCLCPP_INFO(this->get_logger(), "Initialized libviso2 mono odometry with the following parameters: f: %3.3f ,cx: %3.3f , cy: %3.3f", 
      visual_odometer_params_.calib.f,
      visual_odometer_params_.calib.cu,
      visual_odometer_params_.calib.cv);
    }

    // convert image if necessary
    uint8_t *image_data;
    int step;
    cv_bridge::CvImageConstPtr cv_ptr;
    if (image_msg->encoding == sensor_msgs::image_encodings::MONO8)
    {
      image_data = const_cast<uint8_t*>(&(image_msg->data[0]));
      step = image_msg->step;
    }
    else
    {
      cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
      image_data = cv_ptr->image.data;
      step = cv_ptr->image.step[0];
    }

    // run the odometer
    int32_t dims[] = {(int32_t)image_msg->width, (int32_t)image_msg->height, step};
    // on first run, only feed the odometer with first image pair without
    // retrieving data
    if (first_run)
    {
      visual_odometer_->process(image_data, dims);
      tf2::Transform delta_transform;
      delta_transform.setIdentity();
      integrateAndPublish(delta_transform, image_msg->header.stamp);
    }
    else
    {
      bool success = visual_odometer_->process(image_data, dims);
      if(success)
      {
        replace_ = false;
        Matrix camera_motion = Matrix::inv(visual_odometer_->getMotion());
        RCLCPP_DEBUG(this->get_logger(), "Found %i matches with %i inliers.", 
                  visual_odometer_->getNumberOfMatches(),
                  visual_odometer_->getNumberOfInliers());
        //RCLCPP_DEBUG(this->get_logger(), "libviso2 returned the following motion:\n %s", camera_motion);

        tf2::Matrix3x3 rot_mat(
          camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
          camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
          camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2]);
        tf2::Vector3 t(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
        tf2::Transform delta_transform(rot_mat, t);

        integrateAndPublish(delta_transform, image_msg->header.stamp);
      }
      else
      {
        RCLCPP_DEBUG(this->get_logger(), "Call to VisualOdometryMono::process() failed. Assuming motion too small.");
        replace_ = true;
        tf2::Transform delta_transform;
        delta_transform.setIdentity();
        integrateAndPublish(delta_transform, image_msg->header.stamp);
      }

      // create and publish viso2 info msg
      viso2_ros::msg::VisoInfo info_msg;
      info_msg.header.frame_id = image_msg->header.frame_id;
      info_msg.header.stamp = image_msg->header.stamp;
      info_msg.got_lost = !success;
      info_msg.change_reference_frame = false;
      info_msg.num_matches = visual_odometer_->getNumberOfMatches();
      info_msg.num_inliers = visual_odometer_->getNumberOfInliers();
      auto time_elapsed = this->get_clock()->now() - start_time;
      info_msg.runtime = time_elapsed.seconds();
      info_pub_->publish(info_msg);
    }
  }
};

} // end of namespace


int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<viso2_ros::MonoOdometer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

