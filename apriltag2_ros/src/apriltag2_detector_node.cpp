#include <apriltag2_ros/apriltag_detector.h>
#include <ros/ros.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "apriltag2_detector");
  ROS_INFO("\n###################INITIALIZED THE APRILTAG2 DETECTOR###################");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  apriltag2_ros::AprilTagDetector detector(nh, pnh);
  ros::spin();
}
