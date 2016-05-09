#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Twist.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

class Follower
{
public:
	Follower() :
    saveCount(1),
		it(nh),
    rgb_sub( it, "/camera/rgb/image_raw", 1 ),
    depth_sub( it, "/camera/depth/image_raw", 1 ),
    sync( MySyncPolicy( 10 ), rgb_sub, depth_sub )
  {
    sync.registerCallback( boost::bind( &Follower::callback, this, _1, _2 ) );
    image_sub = it.subscribe("/camera/rgb/image_raw", 1, &Follower::driverCallback, this);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    base_cmd.linear.x = 0.2;
  }

  void callback(const ImageConstPtr& rgb_image, const ImageConstPtr& depth_image)
	{
    ROS_INFO("Saving images");
    // Process rgb image
		cv_bridge::CvImagePtr cv_ptr;
  	try
  	{
			cv_ptr = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
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
   		cv_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
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

    saveCount ++;
    //ROS_INFO("Sleep");
    ros::Rate loop_rate(2);
    loop_rate.sleep();
    //ROS_INFO("Wake");
  }

	void saveImage(cv_bridge::CvImagePtr cv_ptr, std::string imageType)
	{
		// Save rgb image to ~/follower_image/rgb
		// Save depth image to ~/follower_image/depth
  	std::ostringstream s;
  	s << "/home/elefly/follower_image/" << imageType << "/" << saveCount << ".png";
  	imwrite(s.str(), cv_ptr->image);
	}

  void driverCallback(const ImageConstPtr& image)
  {
    ROS_INFO("Driver");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      int err = followLineErr(cv_ptr);
      base_cmd.angular.z = -float(err)/200.0;
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
    circle(image, Point(cx, cy),  20, Scalar(0,0,255),  -1);
    imshow("", image);
    waitKey(3);
    return err;
  }

private:
  int saveCount;
  ros::NodeHandle nh;
  image_transport::ImageTransport it;

  typedef image_transport::SubscriberFilter ImageSubscriber;

  ImageSubscriber rgb_sub;
  ImageSubscriber depth_sub;

  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

  Synchronizer< MySyncPolicy > sync;

  image_transport::Subscriber image_sub;
  ros::Publisher cmd_vel_pub;
  geometry_msgs::Twist base_cmd;
};

int main(int argc, char** argv) 
{
  ros::init( argc, argv, "Follower" );
  ROS_INFO("Follower started...");
  Follower f;
  ros::MultiThreadedSpinner s(2);
  while( ros::ok() )
  {
    ros::spin(s);
  }

  return EXIT_SUCCESS;
}
