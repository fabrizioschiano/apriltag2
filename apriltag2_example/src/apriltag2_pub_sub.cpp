#include "node.h"

// Namespaces
using namespace std;



int main(int argc, char** argv)
{
    //    ROS PART
//    cout<< "######################  ROS  ######################"<<endl <<endl;
//    cout<<"-->Initializing ROS..."<<endl;
    ros::init(argc, argv, "apriltag2Node"); // The third element is the name of the node
//    cout<<"-->done!"<<endl;
//    cout<<"-->Initializing NODE..."<<endl;
    cout << "argc:" << argc <<endl;
    cout << "argv:" << argv <<endl;
    apriltag2_detector_ros::Node().spin(argc,argv);
}
