#ifndef NODE_H
#define NODE_H

#include <sstream>
#include <string>

//Basic includes
#include <iostream>

// Opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

// Include for the flycapture, to work with the FLEA camera
#include "flycapture/FlyCapture2.h"

// Include for the apriltags
#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "tag16h5.h"
#include <apriltag2_example/AprilTagDetection.h>
#include <apriltag2_example/AprilTagDetectionArray.h>

// Costum functions (test)
//#include "userDefinedFunctions.h"

// ROS includes
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

#include <Eigen/Core>
#include <tf/transform_datatypes.h>

//#include <boost/signals2/mutex.hpp>
#include "boost/thread/mutex.hpp"

namespace apriltag2_detector_ros {
class Node{
private:
    boost::mutex lock_;
    ros::NodeHandle nh_;
    unsigned long queue_size_;
    //                std::string tracker_config_path_;
    //                std::string model_description_;
    //                std::string color_file_path_;
    //                std::string model_name_;
    //                std::string camera_frame_name_;
    //                bool debug_display_;
    //                std::vector <vpPoint> points;
    //                double dist_point_;
    //                vpBlobsTargetTracker tracker_;
    //                bool status_tracker_;
    //                bool pub_des_pose_;
    //                bool pub_object_cog_;
    //                vpHomogeneousMatrix cMh_d_;
    //                vpHomogeneousMatrix cMh_d_offset;
    bool first_time;
    //                bool move_des_pose;
    //                double d_t;
    //                double d_r;
    //                vpHomogeneousMatrix cMo;

    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat cvI_;
    //                vpImage<unsigned char> I_; // Image used for debug display
    std_msgs::Header image_header_;
    bool got_image_;
    //                vpCameraParameters cam_;
    unsigned int lastHeaderSeq_;
    int freq_;

    void waitForImage();
    void frameCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info);
    const void manageInputKey(const std::string s);
    void computeCog(double p[4][2], double (&returnArray)[2]);
public:
    Node();
    void spin(int argc, char** argv);

};

}

#endif // NODE_H
