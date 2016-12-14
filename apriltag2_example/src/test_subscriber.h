#ifndef TEST_SUBSCRIBER_H
#define TEST_SUBSCRIBER_H
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

class test_subscriber
{
public:
    test_subscriber(ros::NodeHandle &nh);
    ~test_subscriber();
    void spin();
    void getCameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg);

protected:
    // ROS
    ros::NodeHandle m_n;
    std::string m_cameraInfoName;
    int freq;
    ros::Subscriber m_cameraInfoSub;

    //conditions
    bool m_camInfoIsInitialized;
};


#endif // TEST_SUBSCRIBER_H
