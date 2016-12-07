#include "apriltag.h"
#include <ros/ros.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "apriltag_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
//  apriltags_ros::AprilTagDetector detector(nh, pnh);
  apriltag_detector detector(nh,pnh);
  ros::spin();
}
