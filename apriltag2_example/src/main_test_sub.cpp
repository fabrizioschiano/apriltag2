
#include <ros/ros.h>

#include "test_subscriber.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>

int main( int argc, char** argv )
{
  ros::init( argc, argv, "test_subscriber_schiano" );

  ros::NodeHandle n(std::string("~"));

  test_subscriber *node = new test_subscriber(n);

  node->spin();

  delete node;

  return 0;
}
