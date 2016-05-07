#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>

using namespace cv;
using namespace sensor_msgs;

class Driver
{
public:
	Driver()
	{
		image_transport::ImageTransport it(nh);
		image_sub = it.subscribe("/camera/rgb/image_raw", 1, &Driver::callback, this);
		cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
		base_cmd.linear.x = 0.2;
	}

	~Driver()
	{

	}

	void callback(const ImageConstPtr& image)
	{
		cv_bridge::CvImagePtr cv_ptr;
  		try
  		{
    		cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
			int err = followLineErr(cv_ptr);
			base_cmd.angular.z = -float(err)/150.0;
			cmd_vel_pub.publish(base_cmd);
    		//std::cout << "RGB " << cv_ptr->header.stamp << std::endl;
    		//saveImage(cv_ptr, "rgb");
    		//imshow("rgb", cv_ptr->image);
    		//waitKey(10);
  		}

  		catch (cv_bridge::Exception& e)
  		{
    		ROS_ERROR("cv_bridge exception: %s", e.what());
    		return;
  		}
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

private:
	ros::NodeHandle nh;
	image_transport::Subscriber image_sub;
	ros::Publisher cmd_vel_pub;
	geometry_msgs::Twist base_cmd;
};

main(int argc, char** argv)
{
	ros::init(argc, argv, "driver");
  	ROS_INFO("Driver started...");
	Driver driver;
	ros::spin();
}
