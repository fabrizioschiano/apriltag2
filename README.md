# apriltag2

**I am not the author of this algorithm. I am just trying to use it. 
My goal is to use it for a group of quadrotor UAVs equipped with onboard cameras (flea FL3-U3-32S2C).**

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


IROS 2016 PAPER
==================
For more details about the algorithm look here:
https://april.eecs.umich.edu/media/pdfs/wang2016iros.pdf
