#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

class Saver
{
public:
	Saver() :
		it(nh),
    rgb_sub( it, "/camera/rgb/image_raw", 1 ),
    depth_sub( it, "/camera/depth/image_raw", 1 ),
    sync( MySyncPolicy( 10 ), rgb_sub, depth_sub )
  {
    sync.registerCallback( boost::bind( &Saver::callback, this, _1, _2 ) );
  }

  void callback(const ImageConstPtr& rgb_image, const ImageConstPtr& depth_image)
	{
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
  }

	void saveImage(cv_bridge::CvImagePtr cv_ptr, std::string imageType)
	{
		// Save rgb image to ~/follower_image/rgb
		// Save depth image to ~/follower_image/depth
  	std::ostringstream s;
  	s << "/home/elefly/follower_image/" << imageType << "/" << cv_ptr->header.seq << ".png";
  	imwrite(s.str(), cv_ptr->image);
	}

private:
  ros::NodeHandle nh;
  image_transport::ImageTransport it;

  typedef image_transport::SubscriberFilter ImageSubscriber;

  ImageSubscriber rgb_sub;
  ImageSubscriber depth_sub;

  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

  Synchronizer< MySyncPolicy > sync;
};

int main(int argc, char** argv) {
  ros::init( argc, argv, "Saver" );
  ROS_INFO("Saver started...");
  Saver mc;

  while( ros::ok() ){
    ros::spin();
  }

  return EXIT_SUCCESS;
}
