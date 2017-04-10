#include <boost/thread.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>


#include "test_subscriber.h"

//namespace std;

test_subscriber::test_subscriber(ros::NodeHandle &nh)
{
    m_n = nh;

    m_n.param<std::string>("cameraInfoName", m_cameraInfoName, "/camera/camera_info");
    m_n.param( "frequency", freq, 30);
    ROS_INFO("Subscribing to camera_info...");
    m_cameraInfoSub = m_n.subscribe( m_cameraInfoName, 1, (boost::function < void(const sensor_msgs::CameraInfoConstPtr & )>) boost::bind( &test_subscriber::getCameraInfoCb, this, _1 ));
    //    std::string str1 = new std::string;
    ROS_INFO("...done");
    ROS_INFO("Launch test_subscriber node ");

}


test_subscriber::~test_subscriber()
{
    //    delete m_motionProxy;

}


void test_subscriber::getCameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg)
{
    ROS_INFO("Received Camera INFO");
    // Convert the paramenter in the visp format
    //  m_cam = visp_bridge::toVispCameraParameters(*msg);
    //  m_cam.printParameters();


  // Stop the subscriber (we don't need it anymore)
  std::cout << "Stopping Camera INFO subscriber"<<std::endl;
  this->m_cameraInfoSub.shutdown();

  m_camInfoIsInitialized = 1;
}


void test_subscriber::spin()
{
    ros::Rate loop_rate(freq);
    while(ros::ok())
    {

        //    vpMouseButton::vpMouseButtonType button;
        //    bool ret = vpDisplay::getClick(I, button, false);
        //    this->computeHandPose();
        //    if (m_mode_targetCalibration)
        //    {
        //      if (ret && button == vpMouseButton::button1 && m_statusPointArray)
        //        this->computeTargetCalibration();

        //      vpDisplay::flush(I);
        //    }

        //    ret = false;
        ros::spinOnce();
        loop_rate.sleep();
    }
}
