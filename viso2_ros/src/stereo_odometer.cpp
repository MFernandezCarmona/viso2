#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_geometry/stereo_camera_model.h>
#include <cv_bridge/cv_bridge.h>
//#include <pcl_ros/point_cloud.hpp>
//#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <viso_stereo.h>

#include <viso2_ros/msg/viso_info.hpp>

#include "odometer_base.h"
#include "odometry_params.h"

// to remove after debugging
//#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.hpp>

namespace viso2_ros{
    
  using namespace std::chrono_literals;
  // some arbitrary values (0.1m^2 linear cov. 10deg^2. angular cov.)
  static const std::array<double, 36> STANDARD_POSE_COVARIANCE =
  { { 0.1, 0, 0, 0, 0, 0,
      0, 0.1, 0, 0, 0, 0,
      0, 0, 0.1, 0, 0, 0,
      0, 0, 0, 0.17, 0, 0,
      0, 0, 0, 0, 0.17, 0,
      0, 0, 0, 0, 0, 0.17 } };
  static const std::array<double, 36> STANDARD_TWIST_COVARIANCE =
  { { 0.002, 0, 0, 0, 0, 0,
      0, 0.002, 0, 0, 0, 0,
      0, 0, 0.05, 0, 0, 0,
      0, 0, 0, 0.09, 0, 0,
      0, 0, 0, 0, 0.09, 0,
      0, 0, 0, 0, 0, 0.09 } };
  static const std::array<double, 36> BAD_COVARIANCE =
  { { 9999, 0, 0, 0, 0, 0,
      0, 9999, 0, 0, 0, 0,
      0, 0, 9999, 0, 0, 0,
      0, 0, 0, 9999, 0, 0,
      0, 0, 0, 0, 9999, 0,
      0, 0, 0, 0, 0, 9999 } };


  class StereoOdometer : public  OdometerBase {

    private:

      std::shared_ptr<VisualOdometryStereo> visual_odometer_;
      VisualOdometryStereo::parameters visual_odometer_params_;

      //rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
      rclcpp::Publisher<viso2_ros::msg::VisoInfo>::SharedPtr info_pub_;

      bool got_lost_;

      // change reference frame method. 0, 1 or 2. 0 means allways change. 1 and 2 explained below
      int ref_frame_change_method_;
      bool change_reference_frame_;
      double ref_frame_motion_threshold_; // method 1. Change the reference frame if last motion is small
      int ref_frame_inlier_threshold_; // method 2. Change the reference frame if the number of inliers is low

      Matrix reference_motion_;
      // subscriber
      message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_, right_sub_;
      message_filters::Subscriber<sensor_msgs::msg::CameraInfo> left_info_sub_, right_info_sub_;
      typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo> ExactPolicy;
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo> ApproximatePolicy;
      typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
      typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
      std::shared_ptr<ExactSync> exact_sync_;
      std::shared_ptr<ApproximateSync> approximate_sync_;
      int queue_size_;
      
      // for sync checking
      int left_received_, right_received_, left_info_received_, right_info_received_, all_received_;


      // for sync checking
      static void increment(int* value) {
        ++(*value);
      }

      void dataCb(const sensor_msgs::msg::Image::ConstSharedPtr l_image_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr r_image_msg,
                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr l_info_msg,
                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr r_info_msg) {

        // For sync error checking
        ++all_received_;

        // call implementation
        imageCallback(l_image_msg, r_image_msg, l_info_msg, r_info_msg);
      } 


      void checkInputsSynchronized() {
        int threshold = 3 * all_received_;
        if (left_received_ >= threshold || right_received_ >= threshold ||
            left_info_received_ >= threshold || right_info_received_ >= threshold) {
          RCLCPP_WARN(this->get_logger(), "[stereo_processor] Low number of synchronized left/right/left_info/right_info tuples received.\n"
                  "Left images received:       %d (topic '%s')\n"
                  "Right images received:      %d (topic '%s')\n"
                  "Left camera info received:  %d (topic '%s')\n"
                  "Right camera info received: %d (topic '%s')\n"
                  "Synchronized tuples: %d\n"
                  "Possible issues:\n"
                  "\t* stereo_image_proc is not running.\n"
                  "\t  Does `ros2 node info %s` show any connections?\n"
                  "\t* The cameras are not synchronized.\n"
                  "\t  Try restarting the node with parameter _approximate_sync:=True\n"
                  "\t* The network is too slow. One or more images are dropped from each tuple.\n"
                  "\t  Try restarting the node, increasing parameter 'queue_size' (currently %d)",
                  left_received_, left_sub_.getTopic().c_str(),
                  right_received_, right_sub_.getTopic().c_str(),
                  left_info_received_, left_info_sub_.getTopic().c_str(),
                  right_info_received_, right_info_sub_.getTopic().c_str(),
                  all_received_, this->get_name(), queue_size_);
        }
      }


    public:

      StereoOdometer() : 
        OdometerBase("StereoOdometer"),
        // Subscribers
        left_sub_(message_filters::Subscriber<sensor_msgs::msg::Image>( this, "stereo_camera/left/image", rmw_qos_profile_sensor_data)),
        right_sub_(message_filters::Subscriber<sensor_msgs::msg::Image>( this, "stereo_camera/right/image", rmw_qos_profile_sensor_data)),
        left_info_sub_(message_filters::Subscriber<sensor_msgs::msg::CameraInfo>( this, "stereo_camera/left/camera_info")),
        right_info_sub_(message_filters::Subscriber<sensor_msgs::msg::CameraInfo>( this, "stereo_camera/right/camera_info")){





        // internal data
        reference_motion_ = Matrix::eye(4);
        got_lost_ = false;
        change_reference_frame_ = false;
        left_received_ = 0;
        right_received_ = 0;
        left_info_received_ = 0;
        right_info_received_ = 0;
        all_received_ = 0;


        // Resolve topic names
        std::string stereo_ns = "stereo";
        std::string left_topic = "/left/image";
        std::string right_topic = stereo_ns + "/right/image";

        std::string left_info_topic = stereo_ns + "/left/camera_info";
        std::string right_info_topic = stereo_ns + "/right/camera_info";


        // Read local parameters ......................................................................
        odometry_params::loadParams(this, visual_odometer_params_);

        auto transport = this->declare_parameter("transport", "raw");

        // TODO unused??
        // auto ref_frame_change_method_ = this->declare_parameter("ref_frame_change_method", 0);
        // auto ref_frame_motion_threshold_ = this->declare_parameter("ref_frame_motion_threshold", 5.0);
        // auto ref_frame_inlier_threshold_ = this->declare_parameter("ref_frame_inlier_threshold", 150);

        // publishers ......................................................................
        //point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 1);
        info_pub_ = this->create_publisher<viso2_ros::msg::VisoInfo>("info", 1);

        // Subscriptors ......................................................................

        // Complain every 15s if the topics appear unsynchronized
        auto check_synced_timer_ = this->create_wall_timer(15s, std::bind(&StereoOdometer::checkInputsSynchronized, this));

        // Synchronize input topics. Optionally do approximate synchronization.
        auto queue_size_ = this->declare_parameter("queue_size", 5);
        bool approx;
        approx = this->declare_parameter("approximate_sync", false);

        if (approx) {
          approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size_), left_sub_, right_sub_, left_info_sub_, right_info_sub_) );
          approximate_sync_->registerCallback(std::bind(&StereoOdometer::dataCb, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

        } else {
          exact_sync_.reset(new ExactSync(ExactPolicy(queue_size_), left_sub_, right_sub_, left_info_sub_, right_info_sub_) );
          exact_sync_->registerCallback(std::bind(&StereoOdometer::dataCb, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
        }

      }

    protected:

      void initOdometer(
          const sensor_msgs::msg::CameraInfo::ConstSharedPtr l_info_msg,
          const sensor_msgs::msg::CameraInfo::ConstSharedPtr r_info_msg){
        
        //bool approximate_sync;
        //approximate_sync = this->declare_parameter("approximate_sync", false);

        // read calibration info from camera info message
        // to fill remaining parameters
        image_geometry::StereoCameraModel model;
        model.fromCameraInfo(*l_info_msg, *r_info_msg);
        visual_odometer_params_.base      = model.baseline();
        visual_odometer_params_.calib.cu  = model.left().cx();
        visual_odometer_params_.calib.cv  = model.left().cy();
        visual_odometer_params_.calib.f   = model.left().fx();

        visual_odometer_.reset(new VisualOdometryStereo(visual_odometer_params_));
        if (l_info_msg->header.frame_id != "") {
          setSensorFrameId(l_info_msg->header.frame_id);
        }
        // TODO 
        // RCLCPP_INFO(this->get_logger(), "Initialized libviso2 stereo odometry with the following parameters: %d \n  queue_size = %s \n sync = %s \n ref_frame_change_method = %s \n ref_frame_motion_threshold = %s \n ref_frame_inlier_threshold = %s",
        //                 visual_odometer_params_ ,
        //                 queue_size, 
        //                 (approximate_sync? "approximate": "exact"), 
        //                 ref_frame_change_method_, 
        //                 ref_frame_motion_threshold_,  
        //                 ref_frame_inlier_threshold_);
      }

      void imageCallback(
          const sensor_msgs::msg::Image::ConstSharedPtr l_image_msg,
          const sensor_msgs::msg::Image::ConstSharedPtr r_image_msg,
          const sensor_msgs::msg::CameraInfo::ConstSharedPtr l_info_msg,
          const sensor_msgs::msg::CameraInfo::ConstSharedPtr r_info_msg)
      {
        auto start_time = this->get_clock()->now();
        bool first_run = false;
        // create odometer if not exists
        if (!visual_odometer_)
        {
          first_run = true;
          initOdometer(l_info_msg, r_info_msg);
        }

        // convert images if necessary
        uint8_t *l_image_data, *r_image_data;
        uint32_t l_step, r_step;
        cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;
        l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8);
        l_image_data = l_cv_ptr->image.data;
        l_step = l_cv_ptr->image.step[0];
        r_cv_ptr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8);
        r_image_data = r_cv_ptr->image.data;
        r_step = r_cv_ptr->image.step[0];

        rcpputils::assert_true(l_step == r_step);
        rcpputils::assert_true(l_image_msg->width == r_image_msg->width);
        rcpputils::assert_true(l_image_msg->height == r_image_msg->height);

        int32_t dims[] = {(int32_t)l_image_msg->width, (int32_t)l_image_msg->height, (int32_t)l_step};
        // on first run or when odometer got lost, only feed the odometer with
        // images without retrieving data
        if (first_run || got_lost_)
        {
          visual_odometer_->process(l_image_data, r_image_data, dims);
          got_lost_ = false;
          // on first run publish zero once
          if (first_run)
          {
            tf2::Transform delta_transform;
            delta_transform.setIdentity();
            integrateAndPublish(delta_transform, l_image_msg->header.stamp);
          }
        }
        else
        {
          bool success = visual_odometer_->process(
              l_image_data, r_image_data, dims, change_reference_frame_);
          if (success)
          {
            Matrix motion = Matrix::inv(visual_odometer_->getMotion());
            RCLCPP_DEBUG(this->get_logger(), "Found %i matches with %i inliers.",
                      visual_odometer_->getNumberOfMatches(),
                      visual_odometer_->getNumberOfInliers());
            //RCLCPP_DEBUG(this->get_logger(), "libviso2 returned the following motion: %s \n", motion);
            Matrix camera_motion;
            // if image was replaced due to small motion we have to subtract the
            // last motion to get the increment
            if (change_reference_frame_)
            {
              camera_motion = Matrix::inv(reference_motion_) * motion;
            }
            else
            {
              // image was not replaced, report full motion from odometer
              camera_motion = motion;
            }
            reference_motion_ = motion; // store last motion as reference

            tf2::Matrix3x3 rot_mat(
              camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
              camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
              camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2]);
            tf2::Vector3 t(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
            tf2::Transform delta_transform(rot_mat, t);

            setPoseCovariance(STANDARD_POSE_COVARIANCE);
            setTwistCovariance(STANDARD_TWIST_COVARIANCE);

            integrateAndPublish(delta_transform, l_image_msg->header.stamp);

            /*
            if (this->count_subscribers(point_cloud_pub_.getTopic()) > 0){
              computeAndPublishPointCloud(l_info_msg, l_image_msg, r_info_msg,
                                          visual_odometer_->getMatches(),
                                          visual_odometer_->getInlierIndices());
            }
            */
          }
          else
          {
            setPoseCovariance(BAD_COVARIANCE);
            setTwistCovariance(BAD_COVARIANCE);
            tf2::Transform delta_transform;
            delta_transform.setIdentity();
            integrateAndPublish(delta_transform, l_image_msg->header.stamp);

            RCLCPP_DEBUG(this->get_logger(), "Call to VisualOdometryStereo::process() failed.");
            RCLCPP_WARN(this->get_logger(), "10.0 Visual Odometer got lost!");
            got_lost_ = true;
          }

          if(success)
          {

            // Proceed depending on the reference frame change method
            switch ( ref_frame_change_method_ )
            {
              case 1:
              {
                // calculate current feature flow
                double feature_flow = computeFeatureFlow(visual_odometer_->getMatches());
                change_reference_frame_ = (feature_flow < ref_frame_motion_threshold_);
                // TODO
                //RCLCPP_DEBUG(this->get_logger(), "Feature flow is %s ,marking last motion as %s" , feature_flow, (change_reference_frame_ ? "small." : "normal."));
                break;
              }
              case 2:
              {
                change_reference_frame_ = (visual_odometer_->getNumberOfInliers() > ref_frame_inlier_threshold_);
                break;
              }
              default:
                change_reference_frame_ = false;
            }

          }
          else
            change_reference_frame_ = false;

          if(!change_reference_frame_)
            RCLCPP_DEBUG(this->get_logger(), "Changing reference frame");

          // create and publish viso2 info msg
          viso2_ros::msg::VisoInfo info_msg;
          info_msg.header.stamp = l_image_msg->header.stamp;
          info_msg.got_lost = !success;
          info_msg.change_reference_frame = !change_reference_frame_;
          info_msg.num_matches = visual_odometer_->getNumberOfMatches();
          info_msg.num_inliers = visual_odometer_->getNumberOfInliers();
          rclcpp::Duration time_elapsed = this->get_clock()->now() - start_time;
          info_msg.runtime = time_elapsed.seconds();
          info_pub_->publish(info_msg);
        }
      }

      double computeFeatureFlow(
          const std::vector<Matcher::p_match>& matches)
      {
        double total_flow = 0.0;
        for (size_t i = 0; i < matches.size(); ++i)
        {
          double x_diff = matches[i].u1c - matches[i].u1p;
          double y_diff = matches[i].v1c - matches[i].v1p;
          total_flow += sqrt(x_diff * x_diff + y_diff * y_diff);
        }
        return total_flow / matches.size();
      }

/*
      void computeAndPublishPointCloud(
          const sensor_msgs::msg::CameraInfo::ConstSharedPtr& l_info_msg,
          const sensor_msgs::msg::Image::ConstSharedPtr l_image_msg,
          const sensor_msgs::msg::CameraInfo::ConstSharedPtr r_info_msg,
          const std::vector<Matcher::p_match>& matches,
          const std::vector<int32_t>& inlier_indices)
      {
        try
        {
          cv_bridge::CvImageConstPtr cv_ptr;
          cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::RGB8);
          // read calibration info from camera info message
          image_geometry::StereoCameraModel model;
          model.fromCameraInfo(*l_info_msg, *r_info_msg);
          PointCloud::Ptr point_cloud(new PointCloud());
          point_cloud->header.frame_id = getSensorFrameId();
          point_cloud->header.stamp = pcl_conversions::toPCL(l_info_msg->header).stamp;
          point_cloud->width = 1;
          point_cloud->height = inlier_indices.size();
          point_cloud->points.resize(inlier_indices.size());

          for (size_t i = 0; i < inlier_indices.size(); ++i)
          {
            const Matcher::p_match& match = matches[inlier_indices[i]];
            cv::Point2d left_uv;
            left_uv.x = match.u1c;
            left_uv.y = match.v1c;
            cv::Point3d point;
            double disparity = match.u1c - match.u2c;
            model.projectDisparityTo3d(left_uv, disparity, point);
            point_cloud->points[i].x = point.x;
            point_cloud->points[i].y = point.y;
            point_cloud->points[i].z = point.z;
            cv::Vec3b color = cv_ptr->image.at<cv::Vec3b>(left_uv.y,left_uv.x);
            point_cloud->points[i].r = color[0];
            point_cloud->points[i].g = color[1];
            point_cloud->points[i].b = color[2];
          }
          RCLCPP_DEBUG(this->get_logger(), "Publishing point cloud with %zu points.", point_cloud->size());
          point_cloud_pub_->publish(*point_cloud);
        }
        catch (cv_bridge::Exception& e)
        {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
      }
  
 */
 
  }; // end of class

} // end of namespace



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<viso2_ros::StereoOdometer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

