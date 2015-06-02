#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "pddSystem.h"

static const std::string OPENCV_WINDOW = "Processed Image";
static const std::string RGB_NODE = "/head/kinect2/rgb/image"; // Modify this if your ROS node for RGB images is named differently
static const std::string DEPTH_NODE = "/head/kinect2/depth/image"; // Modify this if your ROS node for Depth images is named differently
static const std::string OUTPUT_NODE = "/pdd/output"; // Modify this if you want to rename the output node for the processed images

using namespace pDD;

//static const image_transport::Publisher image_pub;

void callback(const sensor_msgs::ImageConstPtr& RGBImg, const sensor_msgs::ImageConstPtr& DepthImg)
{
  cv_bridge::CvImagePtr RGBcv_ptr;
  cv_bridge::CvImagePtr DEPTHcv_ptr;
  try
  {
    RGBcv_ptr = cv_bridge::toCvCopy(RGBImg, sensor_msgs::image_encodings::BGR8);
    DEPTHcv_ptr = cv_bridge::toCvCopy(DepthImg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //pddSystem::pddSystem pDDS(RGBcv_ptr, DEPTHcv_ptr);

  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, RGBcv_ptr->image);
  cv::waitKey(3);

  //Write Publisher here
  //image_pub.publish(RGBcv_ptr->toImageMsg());

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("Raw RGB", cv_bridge::toCvShare(msg, "bgr8")->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pdd");
  ros::NodeHandle nh;
  // Subscribe to the input RGB and Depth video feeds and then publish output video feed
  message_filters::Subscriber<sensor_msgs::Image> RGB_sub(nh, RGB_NODE, 1);
  message_filters::Subscriber<sensor_msgs::Image> DEPTH_sub(nh, DEPTH_NODE, 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), RGB_sub, DEPTH_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2)); 
  cv::namedWindow(OPENCV_WINDOW);
  cv::startWindowThread();
  cv::namedWindow("Raw RGB");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  //image_transport::ImageTransport _it(nh);
  //image_pub = _it.advertise("/pdd/output_video", 1);
  image_transport::Subscriber sub = it.subscribe(RGB_NODE, 1, imageCallback);
  ros::spin();
  cv::destroyWindow("Raw RGB");
  cv::destroyWindow(OPENCV_WINDOW);
  return 0;
}