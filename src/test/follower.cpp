#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

void saveImage(cv_bridge::CvImagePtr cv_ptr, std::string imageType)
{
// Save rgb image to ~/follower_image/rgb
// Save depth image to ~/follower_image/depth
  std::ostringstream s;
  s << "/home/elefly/follower_image/" << imageType << "/" << cv_ptr->header.seq << ".png";
  imwrite(s.str(), cv_ptr->image);
}

int followLineErr(cv_bridge::CvImagePtr cv_ptr)
{
  Mat image = cv_ptr->image.clone();
  Mat hsv, mask;
  cvtColor(image, hsv, CV_BGR2HSV);
  Scalar lower = Scalar(100, 100, 60);
  Scalar upper = Scalar(179, 255, 255);
  inRange(hsv, lower, upper, mask);
  int search_top = 3*image.rows/4;
  //int search_bot = image.rows;
  mask(Range(0, search_top), Range::all()) = Scalar(0, 0, 0);
  Moments m = moments(mask);
  int cx = m.m10 / m.m00;
  int cy = m.m01 / m.m00;
  int err = cx - image.cols / 2;
  return err;
}

void callback(const ImageConstPtr& image1, const ImageConstPtr& image2)
{
  // Process rgb image...
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::BGR8);
    std::cout << "RGB " << cv_ptr->header.seq << std::endl;
    saveImage(cv_ptr, "rgb");
    //imshow("rgb", cv_ptr->image);
    //waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Process depth image
  try
  {
    cv_ptr = cv_bridge::toCvCopy(image2, sensor_msgs::image_encodings::TYPE_16UC1);
    std::cout << "depth " << cv_ptr->header.seq << std::endl;
    saveImage(cv_ptr, "depth");
    //imshow("depth", cv_ptr->image);
    //waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void test(const sensor_msgs::ImageConstPtr& cv_ptr)
{
  cv_ptr = cv_bridge::toCvCopy(cv_ptr, sensor_msgs::image_encodings::BGR8);
  std::cout << "RGB " << cv_ptr->header.stamp << std::endl;
  imshow("rgb", cv_ptr->image);
  waitKey(10);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "follower");
  ROS_DEBUG("Follower starting...");
  ros::NodeHandle nh;
  message_filters::Subscriber<Image> image1_sub(nh, "/camera/rgb/image_raw", 1);
  message_filters::Subscriber<Image> image2_sub(nh, "/camera/depth/image_raw", 1);

  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
