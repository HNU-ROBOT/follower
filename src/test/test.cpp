#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

class Driver
{
public:
	Driver()
	{
		image_transport::ImageTransport it(nh);
		image_sub = it.subscribe("/camera/rgb/image_raw", 1, &Driver::callback, this);
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
    		std::cout << "RGB " << cv_ptr->header.stamp << std::endl;
    		//saveImage(cv_ptr, "rgb");
    		imshow("rgb", cv_ptr->image);
    		waitKey(10);
  		}

  		catch (cv_bridge::Exception& e)
  		{
    		ROS_ERROR("cv_bridge exception: %s", e.what());
    		return;
  		}
	}

private:
	ros::NodeHandle nh;
	image_transport::Subscriber image_sub;
};

class Saver
{
	typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

public:
	Saver()
	{
		image1_sub.subscribe(nh, "/camera/rgb/image_raw", 1);
		image2_sub.subscribe(nh, "/camera/depth/image_raw", 1);
		// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  		Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
  		sync.registerCallback(boost::bind(&Saver::callback, this, _1, _2));
	}

	~Saver()
	{
		
	}

	void callback(const ImageConstPtr& image1, const ImageConstPtr& image2)
	{
		// Process rgb image...
  		cv_bridge::CvImagePtr cv_ptr;
  		try
  		{
    		cv_ptr = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::BGR8);
    		std::cout << "RGB " << cv_ptr->header.seq << std::endl;
    		//saveImage(cv_ptr, "rgb");
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
    		//saveImage(cv_ptr, "depth");
    		//imshow("depth", cv_ptr->image);
    		//waitKey(10);
  		}
  		catch (cv_bridge::Exception& e)
  		{
    		ROS_ERROR("cv_bridge exception: %s", e.what());
    		return;
  		}
	}

	void saveImage()
	{
	
	}

private:
	ros::NodeHandle nh;
  	message_filters::Subscriber<Image> image1_sub;
  	message_filters::Subscriber<Image> image2_sub;
	
};

main(int argc, char** argv)
{
	ros::init(argc, argv, "follower");
  	ROS_INFO("Follower starting...");
	Driver driver;
	Saver saver;
	ros::spin();
}