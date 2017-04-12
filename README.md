# apriltag2

I am not the author of the apriltag2 algorithm. I am just trying to use it. 
My goal is to use it for a group of quadrotor UAVs equipped with onboard cameras.

ABOUT THE REPOSITORY
=======

**ATTENTION**: This repository is made of 2 parts


**The first part is the one** coming from the folder apriltag-2016-12-01. I downloaded it [here](https://april.eecs.umich.edu/software/apriltag.html). In this part there is basically all the apriltag2 code and it is written in C. 

**The second part is the one** which I wrote and it is needed to publish the apriltag2 data over the ROS network. This part is the one contained in the folder [apriltag2_example](https://github.com/fabrizioschiano/apriltag2/tree/master/apriltag2_example).

Apriltag in brief
----------------
AprilTag is a visual fiducial system, useful for a wide variety of tasks including augmented reality, robotics, and camera calibration. Targets can be created from an ordinary printer, and the AprilTag detection software computes the precise 3D position, orientation, and identity of the tags relative to the camera. Implementations are available in Java, as well as in C. Notably, the C implementation has no external dependencies and is designed to be easily included in other applications, as well as portable to embedded devices. Real-time performance can be achieved even on cell-phone grade processors.

The two main papers to refer to understand the apriltag algorithm are the following:
1. [IROS 2016 - AprilTag 2: Efficient and robust fiducial detection](https://april.eecs.umich.edu/media/pdfs/wang2016iros.pdf): this is the paper which refers to the version of the algorithm that we want to use
2. [ICRA 2011 - AprilTag: A robust and flexible visual fiducial system](http://ieeexplore.ieee.org/abstract/document/5979561/): this is the paper which explains how the first version of the algorithm works

**I am not the author of the apriltag2 algorithm. We are trying to use the apriltag2 algorithm over ROS.** Something like that was already done for the first version of the apriltag algorithm and can be found [here](https://github.com/RIVeR-Lab/apriltags_ros) 

OUR GOAL
=========

Our goal is to use it for a group of quadrotor UAVs equipped with onboard cameras.**


We would like to test the apriltag2 on the following cameras:
1. [flea FL3-U3-32S2C](https://www.ptgrey.com/flea3-32-mp-color-usb3-vision-sony-imx036-camera)
2. We are considering to use also a [basler Dart area scan camera](http://www.baslerweb.com/en/products/cameras/area-scan-cameras/dart/daa1600-60uc).

Specifically we would like to carry out these experiments with bearing information coming directly from the apriltag2 algorithm running on the ODROIDs embedded on the quadrotors

[![ICRA 2017](https://img.youtube.com/vi/uNkMEGOBR0c/0.jpg)](https://www.youtube.com/watch?v=uNkMEGOBR0c "Bearing Rigidity Maintenance for Formations of Quadrotor UAVs - ICRA 2017")

[![IROS 2016](https://img.youtube.com/vi/OqPs3fv0zQg/0.jpg)](https://www.youtube.com/watch?v=OqPs3fv0zQg "A Rigidity-Based Decentralized Bearing Formation Controller for Groups of Quadrotors UAVs - IROS 2016")

The following is a picture of one of the first flights we did with apriltags mounted on our quadrotors:

<img src="https://github.com/fabrizioschiano/apriltag2/blob/master/pictures/apriltagFlying.jpg" width="600"/>

More about me can be found here: http://www.irisa.fr/lagadic/team/Fabrizio.Schiano.html

My goal is to extract a bearing vector from a monocular camera. A bearing vector beta_ij is a unit-norm vector which goes from the robot i to the robot j and it is expressed in the body frame of robot i. In order to do this I would like to use the apriltag2 algorithm. 
All this is driven by the need of detection of multiple quadrotor UAVs through onboard cameras (a flea3 by PointGrey: https://www.ptgrey.com/flea3-32-mp-color-usb3-vision-sony-imx036-camera) in order to retrieve a bearing IJ measurement (i.e. a unit-vector that goes from the origin of the body frame of the robot I to the origin of the body frame of the robot J). 

In the following video you could see how my system is working for now. I am using a flea3 by Pointgrey, converting its image in OpenCV and streaming it (with the camera_info topic) over ROS. Then there is a node which is subscribing to the image+camera_info and applying the apriltag2 algorithm to the image. This node is then publishing the pose of the tag in the camera frame.

Check [HERE](#preliminaryResults)
 for some videos of preliminary results of what I am doing.

This repository is coming from the folder apriltag-2016-10-21. I downloaded it here:
https://april.eecs.umich.edu/software/apriltag.html

AprilTag is a visual fiducial system, useful for a wide variety of tasks including augmented reality, robotics, and camera calibration. Targets can be created from an ordinary printer, and the AprilTag detection software computes the precise 3D position, orientation, and identity of the tags relative to the camera. Implementations are available in Java, as well as in C. Notably, the C implementation has no external dependencies and is designed to be easily included in other applications, as well as portable to embedded devices. Real-time performance can be achieved even on cell-phone grade processors.

INSTALL
=======

The default installation will place headers in /usr/local/include and
shared library in /usr/local/lib. It also installs a pkg-config script
into /usr/local/lib/pkgconfig.

    $ make
    $ sudo make install

To install to a different directory than /usr/local:

    $ PREFIX=/some/path sudo make install


USAGE
=====

A basic AprilTag application can be seen in example/apriltag_demo.c.


Initialization: instantiate a detector and at least one tag family.

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_add_family(td, tf);

Some tag detector parameters can be set at this time.
The default parameters are the recommended starting point.

    td->quad_decimate = 1.0;
    td->quad_sigma = 0.0;
    td->refine_edges = 1;
    td->refine_decode = 0;
    td->refine_pose = 0;

Increase the image decimation if faster processing is required; the
trade-off is a slight decrease in detection range. A factor of 1.0
means the full-size input image is used.

Some Gaussian blur (quad_sigma) may help with noisy input images.


Detection: a single one-line call will process an input image
and return a list of detections.

    zarray_t *detections = apriltag_detector_detect(td, im);

    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        // Do something with det here
    }

    apriltag_detections_destroy(detections);

zarray is a container class which is included with apriltag.
To process through the list of detections, use zarray_get,
as illustrated above.

The caller is responsible for freeing detections by calling
apriltag_detections_destroy().


Cleanup: free the detector and tag family when done.

    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);


OPENCV INTEGRATION
==================

Note that this library has no external dependencies. Most applications
will require, at minimum, a method for acquiring images.

See example/opencv_demo.cc for an example of using AprilTag in C++ with OpenCV.
This example application can be built by executing the following:

    $ cd examples
    $ make opencv_demo

Image data in a cv::Mat object can be passed to AprilTag without creating
a deep copy. Simply create an image_u8_t header for the cv::Mat data buffer:

    cv::Mat img;

    image_u8_t img_header = { .width = img.cols,
        .height = img.rows,
        .stride = img.cols,
        .buf = img.data
    };

Scale
---------------
The tag detector doesn't consider the scale of the tag. Here is some sample code to scale the tag position.

(You need to put, in the following code, in _tagsize_ the size of the tag you are using)

            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);

                matd_t *M = homography_to_pose(det->H, -fx, fy, cx, cy);
                double scale = tagsize / 2.0;
                MATD_EL(M, 0, 3) *= scale;
                MATD_EL(M, 1, 3) *= scale;
                MATD_EL(M, 2, 3) *= scale;
            }

Basically the detector assumes each tag is centered at the origin of its own coordinate system, with the tag corners at (-1, -1), (1, -1), (1, 1), and (-1, 1). Therefore, it assumes the tag's width is 2 units. To get the tag pose in real units, scale the position by (tag size in meters)/2, or whatever unit you prefer.


I think you should use an

ROSRUN OF THE APRILTAG2 EXAMPLE
====================
This section is really under development. 

You should:
 - include apriltag2_example package in the src of your catkin_ws and `catkin_make` and everything was compiled
   I usually do that simply creating a symbolic link in the `src` folder of my catkin_ws. In this way 
   `ln -s /pathToYourFolder/apriltag2_example`
   
   Of course you need to put YOUR path in the `pathToYourFolder`
   
 - Compile the apriltag2 libraries and `sudo make install` them as explained above. If everything works, you should be able to run the following command

`rosrun apriltag2_example apriltag2_exe -f tag16h5` 

and have as an output 

![Alt text](/pictures/aprilTag2_example.png?raw=true "apriltag2_exe ROS output")

this is the output if there is no camera publishing on the topic speicified [HERE](https://github.com/fabrizioschiano/apriltag2/blob/master/apriltag2_example/src/node.cpp#L133
)

<a name="preliminaryResults"></a>
PRELIMINARY RESULTS 
====================

1. Flying with a quadrotor which is recording images and saving them on a SD Card. The apriltag is applied OFFLINE. We are currently working on an ONLINE solution

[![november 2016](https://img.youtube.com/vi/cpYgeW6D_vk/0.jpg)](https://www.youtube.com/watch?v=cpYgeW6D_vk "test1")

2. Figured out how to get a bearing information from the apriltag algorithm

[![december 2016](https://img.youtube.com/vi/javKrPixwNg/0.jpg)](https://www.youtube.com/watch?v=javKrPixwNg "test1")


IROS 2016 PAPER
==================
For more details about the algorithm look here:
https://april.eecs.umich.edu/media/pdfs/wang2016iros.pdf
=======

FLEA CAMERA IMAGE RESOLUTION
====================
The [camera](https://www.ptgrey.com/flea3-32-mp-color-usb3-vision-sony-imx036-camera) we are using has a maximum resolution of `2080x1552`. We chose to do a binning of the image _in hardware on the sensor_ (there is an alternative binning you can do in software through ROS) and in this way the apriltag2 algorithm has to deal with an image which is `1040x776`. The binning is changing the resolution of the image without changing the region of interest (ROI). The 2 different images can be seen below:

- `2080x1552`

 <img src="https://github.com/fabrizioschiano/apriltag2/blob/master/pictures/apriltagResHigh.png" width="300"/>

- `1040x776`

<img src="https://github.com/fabrizioschiano/apriltag2/blob/master/pictures/apriltagResLow.png" width="300"/>

